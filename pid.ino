#include <Wire.h>
#include <Print.h>
#include <FlashStorage_SAMD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define USING_TIMER_TC3 true
#include <SAMDTimerInterrupt.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define MAX_RENDERER_WIDTH 32
#define MAX_RENDERER_BUFFER MAX_RENDERER_WIDTH + 1
#define MAX_RENDERER_HEIGHT 4
#define MAX_NAME_LEN 10
#define MAX_MENU_ITEMS 10
#define MAX_DEPTH 10

#define ENC_CLK_PIN 9
#define ENC_DAT_PIN 8
#define ENC_BUT_PIN 7
#define TEMP_PIN 6
#define POWER_ON_PIN 10

#define CURSOR_CHAR '>'

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(p) Serial.println(p)
#else
#define DEBUG_PRINT(p)
#endif

#define TIMER_IRQ_INTERVAL_MS 5 // how often timer irq fires
#define TEMP_READ_INTERVAL 10 // how often the temp is read
#define TEMP_READ_BUFFER_SIZE 100 // how many temp reads are buffered and averaged
#define PID_INTERVAL_LEN_MS 1000 // total time of the power on loop
#define DISPLAY_REFRESH_INTERVAL 50 // how often display should refresh
#define R_REF 9730 // reference voltage of divider
#define V_REF 4095 // max value of analogRead
#define THERM_R0 9750 // R0 of thermistor
#define THERM_T0 298.15 // T0 of thermistor
#define THERM_B 3950 // Beta value of thermistor

#define PID_FEEDBACK_DURATION 2000 // how often power output should be updated
#define KP_ADD 0
#define KI_ADD 8
#define KD_ADD 16
#define SAVED_ADD 24

volatile int dn = 0;
volatile int n = 0;
volatile int button = 0;
volatile bool refreshDisplay = false;
volatile int power = 0;
volatile float TK = 0;
volatile float r = 0;
volatile float temps[TEMP_READ_BUFFER_SIZE] = {-1};
float temp = 0;
int set = 20;
float Kp = 0;
float Ki = 0;
float Kd = 0;
bool on = false;


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SAMDTimer ITimer(TIMER_TC3);

enum MenuReturn {
  NOTHING,
  SELECT,
  ENTER,
  BACK,
  MAIN,
  RENDER,
};

void readTemp() {
  static int currentInterval = 0;
  int nIntervals = TEMP_READ_INTERVAL / TIMER_IRQ_INTERVAL_MS;
  if (!currentInterval) {
    uint8_t i;
    for (i = TEMP_READ_BUFFER_SIZE - 1; i > 0; i--) {
      temps[i] = temps[i - 1];
    }
    unsigned int vRaw = (unsigned int)analogRead(TEMP_PIN);
    r = R_REF * ((float)V_REF/vRaw - 1);
    float rInf = THERM_R0 * exp(-THERM_B/THERM_T0);
    temps[0] = THERM_B / (log(r/rInf));
    float sum = 0;
    for (i = 0; i < TEMP_READ_BUFFER_SIZE; i++) {
      if (temps[i] < 0) {
        i--;
        break;
      }
      sum += temps[i];
    }
    TK = sum / i;
  }
  currentInterval = (currentInterval + 1) % nIntervals;
}

void save() {
  EEPROM.put(KP_ADD, Kp);
  EEPROM.put(KI_ADD, Ki);
  EEPROM.put(KD_ADD, Kd);
  EEPROM.commit();
}

void load() {
  EEPROM.get(KP_ADD, Kp);
  EEPROM.get(KI_ADD, Ki);
  EEPROM.get(KD_ADD, Kd);
}

void handleDisplayRefresh() {
  static int currentInterval = 1;
  int nIntervals = DISPLAY_REFRESH_INTERVAL / TIMER_IRQ_INTERVAL_MS;
  if (!currentInterval) refreshDisplay = true;
  currentInterval = (currentInterval + 1) % nIntervals;
}

void handlePowerOutput() {
  power = power > 100 ? 100 : power;
  power = power < 0 ? 0 : power;
  static int currentInterval = 2;
  static byte on = HIGH;
  int nIntervals = PID_INTERVAL_LEN_MS / TIMER_IRQ_INTERVAL_MS;
  int intervalPercent = currentInterval * 100 / nIntervals;
  if (intervalPercent >= power && on == LOW) {
    on = HIGH;
    digitalWrite(LED_BUILTIN, on);
    digitalWrite(POWER_ON_PIN, !on);
  } else if (intervalPercent < power && on == HIGH ) {
    on = LOW;
    digitalWrite(LED_BUILTIN, on);
    digitalWrite(POWER_ON_PIN, !on);
  }
  currentInterval = (currentInterval + 1) % nIntervals;
}

void calculatePower() {
  static bool prevOn = false;
  static int prevSet = set;
  static int currentInterval = 3;
  static float cumulativeErr = 0;
  static float prevErr = 0;
  int nIntervals = PID_FEEDBACK_DURATION / TIMER_IRQ_INTERVAL_MS;
  if (on) {
    if (prevSet != set) {
      cumulativeErr = 0;
      prevErr = 0;
      prevSet = set;
    }
    if (currentInterval == 0) {
      float currentErr = set - (TK - 273.15);
      float dErr = (currentErr - prevErr) / ((float)PID_FEEDBACK_DURATION/1000);
      if (!prevOn) {
        dErr = 0;
      }
      prevErr = currentErr;
      cumulativeErr += currentErr * ((float)PID_FEEDBACK_DURATION/1000);
      power = Kp * currentErr + Ki * cumulativeErr + Kd * dErr;
    }
  } else {
    power = 0;
    cumulativeErr = 0;
    prevErr = 0;
  }
  currentInterval = (currentInterval + 1) % nIntervals;
  prevOn = on;
}

void timerRoutine() {
  handlePowerOutput();
  handleDisplayRefresh();
  readTemp();
  calculatePower();
}

class Renderer {
  public:
    Renderer(uint8_t nLines) : nLines_(nLines), viewPos_(0), scrollPos_(0) {}
    virtual void render(char lines[MAX_MENU_ITEMS][MAX_RENDERER_BUFFER], uint8_t nItems) = 0;
    void scrollUp() {
      DEBUG_PRINT("Renderer scrollUp");
      if (scrollPos_ == 0) {
        if (viewPos_ > 0) {
          viewPos_--;
        }
      } else {
        scrollPos_--;
      }
    }
    void scrollDown(uint8_t nItems) {
      DEBUG_PRINT("Renderer scrollDown");
      // scrollPos not at end of screen
      if (scrollPos_ == nLines_ - 1) {
        // and not at end of view move view down
        if (viewPos_ < nItems - nLines_) {
          viewPos_++;
        }
      // if scrollPos not at end of screen
      } else {
        // scroll down
        scrollPos_++;
      }
    }
  protected:
    uint8_t nLines_;
    uint8_t viewPos_;
    uint8_t scrollPos_;
};

uint8_t _dumbPow(uint8_t base, uint8_t ex) {
  if (ex == 0) {
    return 1;
  } else if (ex == 1) {
    return base;  
  } else {
    for (uint8_t i = 0; i < ex - 1; i++) {
      base = base * base;
    }
  }
  return base;
}

class SSD1306Renderer32x128 : public Renderer {
  public:
    SSD1306Renderer32x128(Adafruit_SSD1306* display, uint8_t textSize) : \
      Renderer(4 / _dumbPow(2, textSize - 1)), \
      display_(display), \
      textSize_(textSize) {}
    void render(char lines[MAX_MENU_ITEMS][MAX_RENDERER_BUFFER], uint8_t nItems) {
      DEBUG_PRINT("Renderer render");
      display_->clearDisplay();
      display_->setCursor(0, 0);
      display_->setTextSize(textSize_);
      //if (viewPos_ > nItems - nLines_ - 1) {
      //  DEBUG_PRINT("HMMM");
      //  viewPos_ = nItems - nLines_ - 1;
      //}
      for (uint8_t i = 0; i < nItems && i < nLines_; i++) {
        DEBUG_PRINT("Printing");
        DEBUG_PRINT(lines[viewPos_ + i]);
        DEBUG_PRINT("View Pos:");
        DEBUG_PRINT(viewPos_);
        display_->println(lines[viewPos_ + i]);
      }
      display_->display();
    }
  private:
    Adafruit_SSD1306* display_;
    uint8_t textSize_;
};



class MenuItem {
  public:
    MenuItem(const char* name, Renderer* renderer, bool dynamic, bool dynamicPreview) : \
      name_(name), \
      renderer_(renderer), \
      dynamic_(dynamic), \
      dynamicPreview_(dynamicPreview) {}
    virtual MenuReturn up();
    virtual MenuReturn down();
    virtual MenuReturn button();
    virtual void render();
    virtual MenuReturn select();
    virtual void preview(char s[MAX_RENDERER_BUFFER], uint8_t previewLen);
    virtual MenuItem* getItem();
    const char* getName() { return name_; }
    bool isDynamic() { return dynamic_; }
    bool hasDynamicPreview() { return dynamicPreview_; }
  protected:
    const char* name_;
    Renderer* renderer_;
    char lines_[MAX_MENU_ITEMS][MAX_RENDERER_BUFFER];
    bool dynamic_;
  private:
    bool dynamicPreview_;
};



class OutputValue: public MenuItem {
  public:
    OutputValue(const char* name, Renderer* renderer, void* value) : MenuItem(name, renderer, true, true), v_(value) {}
    MenuReturn up() { return NOTHING; }
    MenuReturn down() { return NOTHING; }
    MenuReturn button() { return NOTHING; }
    MenuReturn select() { return NOTHING; }
    void render() {}
    MenuItem* getItem() { return NULL; }  
  protected:
    void* v_;
};

class OutputFloatValue : public OutputValue {
  public:
    OutputFloatValue(const char* name, Renderer* renderer, float* value) : OutputValue(name, renderer, (void*)value) {}
    void preview(char s[MAX_RENDERER_BUFFER], uint8_t previewLen) {
      snprintf(s, previewLen, "%s: %.3f", name_, *(float*)v_);
    }
};

class OutputVolIntValue : public OutputValue {
  public:
    OutputVolIntValue(const char* name, Renderer* renderer, volatile int* value) : OutputValue(name, renderer, (void*)value) {}
    void preview(char s[MAX_RENDERER_BUFFER], uint8_t previewLen) {
      snprintf(s, previewLen, "%s: %d", name_, *(int*)v_);
    }
};

class IntValue : public MenuItem {
  public:
    IntValue(const char* name, Renderer* renderer, int* value) : MenuItem(name, renderer, false, false), v_(value) {}
    MenuReturn up() { (*v_)++; render(); return NOTHING; }
    MenuReturn down() { (*v_)--; render(); return NOTHING; }
    MenuReturn select() { return ENTER; }
    MenuReturn button() { return BACK; }
    MenuItem* getItem() { return NULL; }
    void preview(char s[MAX_RENDERER_BUFFER], uint8_t previewLen) {
      uint8_t max_size = min(previewLen, MAX_RENDERER_BUFFER);
      snprintf(s, max_size, "%s: %d", name_, *v_);
    }
    void render() {
      char lines[MAX_MENU_ITEMS][MAX_RENDERER_BUFFER];
      snprintf(lines[0], MAX_RENDERER_BUFFER, "%d", *v_);
      renderer_->render(lines, 1);
    }
    int getValue() { return *v_; }
  protected:
    int* v_;
};

class FloatValue : public MenuItem {
  public:
    FloatValue(const char* name, Renderer* renderer, float* value, uint8_t nDigits, uint8_t decimalLocation) : 
      MenuItem(name, renderer, false, false),
      v_(value),
      nDigits_(nDigits),
      decimalLocation_(decimalLocation),
      currentDigit_(0),
      currentDigitIndex_(0),
      tempValue_(0),
      digits_({0}) {}
    MenuReturn up() { 
      if (negIndex_) {
        neg_ = !neg_;
      } else {
        currentDigit_ = (currentDigit_ + 1) % 10;
      }
      render();
      return NOTHING;
    }
    
    MenuReturn down() {
      if (negIndex_) {
        neg_ = !neg_;
      } else {
        currentDigit_ = currentDigit_ == 0 ? 9 : currentDigit_ - 1;
      }
      render();
      return NOTHING;
    }
    
    MenuReturn select() {
      neg_ = *v_ < 0;
      negIndex_ = true;
      float tmp = abs(*v_);
      currentDigit_ = 0;
      tempValue_ = 0;
      currentDigitIndex_ = 0;
      DEBUG_PRINT("Select digits:");
      for(int i = 0; i < nDigits_; i++) {
        float ten = pow(10, nDigits_ - decimalLocation_ - i - 2);
        uint8_t digit = floor(tmp / ten);
        DEBUG_PRINT(digit);
        tmp -= ten * digit;
        digits_[i] = digit;
      }
      return ENTER;
    }
    
    MenuReturn button() {
      if (negIndex_) {
        negIndex_ = false;
      } else {
        DEBUG_PRINT("Power:");
        DEBUG_PRINT(nDigits_ - decimalLocation_ - currentDigitIndex_ - 2);
        tempValue_ += currentDigit_ * pow(10, nDigits_ - decimalLocation_ - currentDigitIndex_ - 2);
        DEBUG_PRINT(tempValue_);
        digits_[currentDigitIndex_] = currentDigit_;
        currentDigitIndex_++;
        currentDigit_ = digits_[currentDigitIndex_];
        if (currentDigitIndex_ >= nDigits_) {
          *v_ = neg_ ? tempValue_ * -1 : tempValue_;
          return BACK;
        }
      }
      return NOTHING;
    }
    
    void render() {
      char s[nDigits_ + 3];
      char _s[2];
      uint8_t decimalPrinted = 0;
      if (neg_) {
        s[0] = '-';
      } else {
        s[0] = '+';
      }
      for(uint8_t i = 0; i < nDigits_; i++) {
        if (i == currentDigitIndex_) {
          snprintf(_s, 2, "%d", currentDigit_);
        } else {
          snprintf(_s, 2, "%d", digits_[i]);
        }
        if (i == decimalLocation_) {
          s[i + 1] = '.';
          decimalPrinted = 1;
        }
        s[i + 1 + decimalPrinted] = _s[0];
      }
      s[nDigits_ + 2] = '\0';
      char lines[MAX_MENU_ITEMS][MAX_RENDERER_BUFFER];
      snprintf(lines[0], MAX_RENDERER_BUFFER, "%s", s);
      renderer_->render(lines, 1);
    }
    void preview(char s[MAX_RENDERER_BUFFER], uint8_t previewLen) {
      uint8_t max_size = min(previewLen, MAX_RENDERER_BUFFER);
      snprintf(s, max_size, "%s: %.3f", name_, *v_);
    }
    MenuItem* getItem() {
      return NULL;
    }
  private:
    float* v_;
    float tempValue_;
    bool neg_;
    bool negIndex_;
    uint8_t currentDigit_;
    uint8_t nDigits_;
    uint8_t decimalLocation_;
    uint8_t currentDigitIndex_;
    uint8_t digits_[256];
};

class BoundedIntValue : public IntValue {
  public:
    BoundedIntValue(const char* name, Renderer* renderer, int* value, int lowerBound, int upperBound) : \
      IntValue(name, renderer, value), \
      lowerBound_(lowerBound), \
      upperBound_(upperBound) {}
    MenuReturn up() { *v_ = *v_ < upperBound_ ? (*v_) + 1 : *v_; render(); return NOTHING; }
    MenuReturn down() { *v_ = *v_ > lowerBound_ ? (*v_) - 1 : *v_; render(); return NOTHING; }
  private:
    int lowerBound_;
    int upperBound_;
};

class DummyItem : public MenuItem {
  public:
    DummyItem(const char* name, Renderer* renderer) : \
      MenuItem(name, renderer, false, false) {}
    MenuReturn up() { return NOTHING; }
    MenuReturn down() { return NOTHING; }
    MenuReturn button() { return NOTHING; }
    MenuReturn select() { return NOTHING; }
    MenuItem* getItem() { return NULL; }
    void render() {};
    void preview(char s[MAX_RENDERER_BUFFER], uint8_t previewLen) {
      snprintf(s, min(MAX_RENDERER_BUFFER, previewLen), "%s", name_);
    }
};

class Save : public DummyItem {
  public:
    Save(const char* name, Renderer* renderer) : \
      DummyItem(name, renderer) {}
    MenuReturn select() {
      save();
      return NOTHING;
    }
};

class Load : public DummyItem {
  public:
    Load(const char* name, Renderer* renderer) : \
      DummyItem(name, renderer) {}
    MenuReturn select() {
      load();
      return RENDER;
    }
};

class ToggleItem : public DummyItem {
  public:
    ToggleItem(const char* name, Renderer* renderer, void* value, const char* trueText, const char* falseText) : DummyItem(name, renderer), trueText_(trueText), falseText_(falseText), value_((bool*)value) {}
    MenuReturn select() { *value_ = !(*value_); return RENDER; }
    void preview(char s[MAX_RENDERER_BUFFER], uint8_t previewLen) {
      snprintf(s, min(MAX_RENDERER_BUFFER, previewLen), "%s", *value_ ? trueText_ : falseText_);
    }
  private:
    bool* value_;
    const char* trueText_;
    const char* falseText_;
};

class BackButton : public DummyItem {
  public:
    BackButton(Renderer* renderer) : DummyItem("Back", renderer) {}
    MenuReturn select() { return BACK; }
};

class ScrollMenu : public MenuItem {
  public:
    ScrollMenu(const char* name, Renderer* renderer, MenuItem* items[MAX_MENU_ITEMS], uint8_t nItems) : \
      MenuItem(name, renderer, false, false), \
      items_(items), \
      nItems_(nItems), \
      cursorPos_(0) {
        for(uint8_t i = 0; i < nItems; i++) {
          if (items[i]->hasDynamicPreview()) dynamic_ = true;
        }
    }
    MenuReturn up() {
      DEBUG_PRINT("ScrollMenu up");
      DEBUG_PRINT(cursorPos_);
      if (cursorPos_ > 0) {
        cursorPos_--;
        renderer_->scrollUp();
        render();
      }
      return NOTHING;
    }
    MenuReturn down() {
      DEBUG_PRINT("ScrollMenu down");
      DEBUG_PRINT(cursorPos_);
      if (cursorPos_ < nItems_ - 1) {
        cursorPos_ ++;
        renderer_->scrollDown(nItems_);
        render();
      }
      return NOTHING;
    }
    MenuReturn select() {
      return ENTER;
    }
    MenuReturn button() {
      DEBUG_PRINT("ScrollMenu button");
      return SELECT;
    }
    MenuItem* getItem() {
      return items_[cursorPos_];
    }
    void preview(char s[MAX_RENDERER_BUFFER], uint8_t previewLen) {
      snprintf(s, min(previewLen, MAX_RENDERER_BUFFER), "%s", name_);
    }
    void render() {
      DEBUG_PRINT("ScrollMenu render");
      refresh();
      renderer_->render(lines_, nItems_);
    }
    bool isDynamic() {
      for (uint8_t i = 0; i < nItems_; i++) {
        if (items_[i]->hasDynamicPreview()) return true;
      }
      return false;
    }
  protected:
    MenuItem** items_;
    uint8_t nItems_;
    uint8_t cursorPos_;
  private:
    void refresh() {
      for (uint8_t i = 0; i < nItems_; i++) {
        char cursor = ' ';
        if (i == cursorPos_) cursor = CURSOR_CHAR;
        char linePreview[MAX_RENDERER_BUFFER - 1];
        items_[i]->preview(linePreview, MAX_RENDERER_BUFFER - 1);
        snprintf(lines_[i], MAX_RENDERER_BUFFER, "%c%s", cursor, linePreview);
        DEBUG_PRINT("Preview:");
        DEBUG_PRINT(linePreview);
        DEBUG_PRINT(lines_[i]);
      }
    }
};

class Menu {
  public:
    Menu(MenuItem* topItem) : stackLen_(1) { stack_[0] = topItem; }
    void up() {
      DEBUG_PRINT("Menu up");
      act(stack_[0], stack_[0]->up());
    }
    void down() {
      DEBUG_PRINT("Menu down");
      act(stack_[0], stack_[0]->down());
    }
    void button() {
      DEBUG_PRINT("Menu button");
      MenuReturn r = stack_[0]->button();
      act(stack_[0], r);
    }
    void render() {
      DEBUG_PRINT("Menu render");
      stack_[0]->render();
    }
    void refresh() {
      if (stack_[0]->isDynamic()) {
        render();
      }
    }
  private:
    MenuItem* stack_[MAX_DEPTH];
    uint8_t stackLen_;
    bool pushStack(MenuItem* item) {
      DEBUG_PRINT("Push stack");
      DEBUG_PRINT(stackLen_);
      if (stackLen_ == MAX_DEPTH) {
        return false;
      }
      for(uint8_t i = stackLen_; i > 0; i--) {
        stack_[i] = stack_[i - 1];
      }
      stack_[0] = item;
      stackLen_++;
      return true;
    }

    bool popStack() {
      if (stackLen_ == 1) {
        return false;
      } else {
        for (uint8_t i = 0; i < stackLen_ - 1; i++) {
          stack_[i] = stack_[i + 1];
        }
        stackLen_--;
        return true;
      }
    }

    void act(MenuItem* item, MenuReturn r) {
      MenuItem* i = item->getItem();
      DEBUG_PRINT("Act");
      switch (r) {
        case SELECT:
          DEBUG_PRINT("Select");
          if (i != NULL) {
            act(i, i->select());
          }
          break;
        case ENTER:
          DEBUG_PRINT("Enter");
          if (pushStack(item)) stack_[0]->render();
          break;
        case BACK:
          DEBUG_PRINT("Back");
          popStack();
          stack_[0]->render();
          break;
        case MAIN:
          break;
        case NOTHING:
          break;
        case RENDER:
          render();
          break;
      }
    }
};

Menu* menu;

void doInterruptThing() {
  static unsigned long last_millis = 0;
  unsigned long m = millis();
  int n_up = 0;
  for (uint8_t i = 0; i < 100; i++) {
    if (!digitalRead(ENC_CLK_PIN)) return;
  }
  if (m - last_millis > 0) {
    for (uint8_t i = 0; i < 1; i++) {
      if (!digitalRead(ENC_DAT_PIN)) {
        n_up++;
      } else {
        n_up--;
      }
    }
    if (n_up > 0) {
      dn++; 
    } else {
      dn--;
    }
  }
  last_millis = m;
}

void buttonInt() {
  static unsigned long prev = 0;
  for (uint8_t i = 0; i < 100; i++) {
    if (!digitalRead(ENC_BUT_PIN)) return;
  }
  unsigned long t = millis();
  if (t - prev < 10) return;
  prev = t;
  button = 1;
}

int scroll_n() {
  static unsigned long t = 0;
  static int prev_dn = 0;
  if (dn != 0) {
    unsigned long new_t = millis();
    // if direction changes
    if ((prev_dn < 0) == (dn > 0)) {
      // only allow direction changes after a certain amount of time (sometimes the encoder bounces back 1 after scrolling)
      if (new_t - t < 50) {
        dn = 0;
        return 0;
      }
    }
    prev_dn = dn;
    t = new_t;
    dn = 0;
    return prev_dn;
  }
  return 0;
}

void setup() {
  // blink until serial connected while in debug mode
#ifdef DEBUG
  while (!Serial) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);                       // wait for a second
  }
#endif
  EEPROM.setCommitASAP(false);
  Serial.begin(115200);
  Serial.println("Hi");
  analogReadResolution(12);
  // Setup the display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  // display.display();
  // delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.display();
  static SSD1306Renderer32x128 r1(&display, 1);
  static SSD1306Renderer32x128 r2(&display, 3);
  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DAT_PIN, INPUT_PULLUP);
  pinMode(ENC_BUT_PIN, INPUT);
  pinMode(TEMP_PIN, INPUT);
  pinMode(POWER_ON_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(100);
  attachInterrupt(digitalPinToInterrupt(ENC_BUT_PIN), buttonInt, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), doInterruptThing, RISING);
  if (ITimer.attachInterruptInterval_MS(TIMER_IRQ_INTERVAL_MS, timerRoutine)) {
    DEBUG_PRINT("Timer attached");
  }

  static int bVal = 1;
  static int cVal = 2;
  static OutputVolIntValue p("Power", NULL, &power);
  static OutputFloatValue t_("Temp C", NULL, &temp);
  static BoundedIntValue s("Set Point C", &r2, &set, 20, 80);
  static FloatValue a("Kp", &r2, &Kp, 5, 2);
  static FloatValue b("Ki", &r2, &Ki, 5, 2);
  static FloatValue c("Kd", &r2, &Kd, 5, 2);
  static ToggleItem t("power", NULL, &on, "Power: on", "Power: off");
  static Save saveButton("Save", NULL);
  static Load loadButton("Load", NULL);
  static MenuItem* items[] = {&p, &t_, &s, &a, &b, &c, &t, &saveButton, &loadButton};
  static ScrollMenu top("Top", &r1, items, 9);
  static Menu _menu(&top);
  menu = &_menu;
  menu->render();
  Serial.println("Done setup");
}

void loop() {
  // put your main code here, to run repeatedly:
    static byte a = HIGH;
    int _dn = scroll_n();
    byte _button = button;
    button = 0;
    temp = (TK - 273.15);// * (9.0/5.0) + 32;
    //temp = TK;
    //test = r;
    if (_button) {
      menu->button();
      _button=0;
    }
    if (refreshDisplay) {
      menu->refresh();
    }
    refreshDisplay = false;
    if (_dn == 0) {
      return;
    }
    if (_dn > 0) {
      menu->up();
    } else {
      menu->down();
    }
    dn = 0;
    //display.display();*/
  
}
