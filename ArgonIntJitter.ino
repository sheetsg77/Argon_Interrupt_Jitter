// state codes
const int DHTLIB_OK = 0;
const int DHTLIB_ACQUIRING = 1;
const int DHTLIB_ACQUIRED = 2;
const int DHTLIB_RESPONSE_OK = 3;

// error codes
const int DHTLIB_ERROR_CHECKSUM = -1;
const int DHTLIB_ERROR_ISR_TIMEOUT = -2;
const int DHTLIB_ERROR_RESPONSE_TIMEOUT = -3;
const int DHTLIB_ERROR_DATA_TIMEOUT = -4;
const int DHTLIB_ERROR_ACQUIRING = -5;
const int DHTLIB_ERROR_DELTA = -6;
const int DHTLIB_ERROR_NOTSTARTED = -7;

class PDHT
{
public:
  PDHT(uint8_t sigPin);
  void begin();
  int getStatus();
  int acquire();
  bool acquiring();
  volatile uint8_t _edges[45];

private:
  void _isrCallback();
  void detachISRIfRequested();
  volatile bool _detachISR;
  volatile unsigned long _us;
  volatile uint8_t *_e;
  volatile int _status;
  volatile uint8_t _idx;
  enum states

  {
    RESPONSE = 0,
    DATA = 1,
    ACQUIRED = 2,
    STOPPED = 3,
    ACQUIRING = 4
  };
  volatile states _state;
  int _sigPin;
  unsigned long _lastreadtime;
};

PDHT::PDHT(uint8_t sigPin)
{
  _sigPin = sigPin;
}

void PDHT::begin()
{
  _lastreadtime = 0;
  _state = STOPPED;
  _status = STOPPED;
  _detachISR = false;
  pinMode(_sigPin, OUTPUT);
  digitalWrite(_sigPin, HIGH);
  //delay(1000);
}

int PDHT::acquire()
{
  unsigned long currenttime = millis();
  if (_state == STOPPED || _state == ACQUIRED)
  {
    _lastreadtime = currenttime;
    _status = DHTLIB_ACQUIRING;
    _state = ACQUIRING;
    for (int i = 0; i < 44; i++)
      _edges[i] = 0;
    _e = &_edges[0];
    _idx = 0;

    pinMode(_sigPin, OUTPUT);
    digitalWrite(_sigPin, LOW);
    delayMicroseconds(1500);
    pinMode(_sigPin, INPUT);
    //delayMicroseconds(80);
    _us = micros();
    _detachISR = false;

    attachInterrupt(_sigPin, &PDHT::_isrCallback, this, FALLING, 13);
    return DHTLIB_ACQUIRING;
  }
  else
    return DHTLIB_ERROR_ACQUIRING;
}

void PDHT::_isrCallback()
{
  if (_detachISR)
    return;
  unsigned long newUS = micros();
  unsigned long delta = (newUS - _us);
  _us = newUS;

  if (delta > 6000)
  {
    _status = DHTLIB_ERROR_ISR_TIMEOUT;
    _state = STOPPED;
    _detachISR = true;
    return;
  }
  *_e++ = delta;
  if (++_idx == 41)
  { // collect 41 bits
    _detachISR = true;
    _status = DHTLIB_OK;
    _state = ACQUIRED;
  }
}

bool PDHT::acquiring()

{
  if (_state != ACQUIRED && _state != STOPPED)
    return true;
  // else
  detachISRIfRequested();
  return false;
}

int PDHT::getStatus()
{
  detachISRIfRequested();
  return _status;
}

void PDHT::detachISRIfRequested()
{
  if (_detachISR)
  {
    detachInterrupt(_sigPin);
    _detachISR = false;
  }
}

PDHT DHTA(D4);
PDHT DHTB(D5);

void setup()
{
  Serial.begin(115200);
  Serial.println("PDHT 2 Sensor program DHT.acquire and DHT.aquiring");

  DHTA.begin();
  DHTB.begin();

  delay(1000); // Delay 1s to let the sensors settle
}

void printEdgeTiming(class PDHT *_d)
{
  byte n;
  uint8_t bits[5];
  uint8_t cnt = 7;
  uint8_t idx = 0;
  uint8_t delta;
  uint8_t sum;
  uint8_t target;
  uint8_t resid;
  volatile uint8_t *_e = &_d->_edges[0];

  for (n = 0; n < 5; n++)
    bits[n] = 0;
  cnt = 7;
  idx = 0;
  sum = 0;
  resid = 0;
  target = 0;

  for (n = 0; n < 41; n++)
  {
    delta = *_e++;
    if (n == 0 && delta > 191)
      resid = delta - 191;
    else
      resid = 0;
    if (n > 0)
    {
      bits[idx] <<= 1;
      if (delta > 105)
      {
        bits[idx] |= 1;
        if (cnt == 7)
        {
          target = 142;
        }
        else
        {
          target = 128;
        }
      }
      else
      {
        if (cnt == 7)
          target = 94;
        else
          target = 80;
      }
      if (delta > target)
        resid = delta - target;
      else
        resid = 0;
      if (cnt == 0)
      {
        cnt = 7;
        if (++idx == 5)
        {
          sum = bits[0] + bits[1] + bits[2] + bits[3];
        }
      }
      else
      {
        cnt--;
      }
    }
    Serial.print(delta);
    Serial.print(" ");
  }
  for (n = 0; n < 5; n++)
  {
    Serial.print(bits[n]);
    Serial.print(" ");
  }
  Serial.print(sum);
  Serial.print(" ");
  Serial.print(bits[4]);
  if (sum == bits[4])
    Serial.print(" ok");
  else
    Serial.print(" CkSumErr");
  Serial.print("\n\r");
}

void printEdgeTimingResid(class PDHT *_d)
{
  byte n;
  uint8_t bits[5];
  uint8_t cnt = 7;
  uint8_t idx = 0;
  uint8_t delta;
  uint8_t sum;
  uint8_t target;
  uint8_t resid;
  volatile uint8_t *_e = &_d->_edges[0];

  for (n = 0; n < 5; n++)
    bits[n] = 0;
  cnt = 7;
  idx = 0;
  sum = 0;
  resid = 0;
  target = 0;

  for (n = 0; n < 41; n++)
  {
    delta = *_e++ + resid;
    if (n == 0 && delta > 191)
      resid = delta - 191;
    else
      resid = 0;
    if (n > 0)
    {
      bits[idx] <<= 1;
      if (delta > 105)
      {
        bits[idx] |= 1;
        if (cnt == 7)
        {
          target = 142;
        }
        else
        {
          target = 128;
        }
      }
      else
      {
        if (cnt == 7)
          target = 94;
        else
          target = 80;
      }
      if (delta > target)
        resid = delta - target;
      else
        resid = 0;
      if (cnt == 0)
      {
        cnt = 7;
        if (++idx == 5)
        {
          sum = bits[0] + bits[1] + bits[2] + bits[3];
        }
      }
      else
      {
        cnt--;
      }
    }
    Serial.print(delta);
    Serial.print(" ");
  }
  for (n = 0; n < 5; n++)
  {
    Serial.print(bits[n]);
    Serial.print(" ");
  }
  Serial.print(sum);
  Serial.print(" ");
  Serial.print(bits[4]);
  if (sum == bits[4])
    Serial.print(" ok");
  else
    Serial.print(" CkSumErr");
  Serial.print("\n\r");
}

void printSensorData(class PDHT *_d)
{
  int result = _d->getStatus();
  /*
  switch (result)
  {
  case DHTLIB_OK:
    Serial.println("OK");
    break;
  case DHTLIB_ERROR_CHECKSUM:
    Serial.println("Error\n\r\tChecksum error");
    break;
  case DHTLIB_ERROR_ISR_TIMEOUT:
    Serial.println("Error\n\r\tISR time out error");
    break;
  case DHTLIB_ERROR_RESPONSE_TIMEOUT:
    Serial.println("Error\n\r\tResponse time out error");
    break;
  case DHTLIB_ERROR_DATA_TIMEOUT:
    Serial.println("Error\n\r\tData time out error");
    break;
  case DHTLIB_ERROR_ACQUIRING:
    Serial.println("Error\n\r\tAcquiring");
    break;
  case DHTLIB_ERROR_DELTA:
    Serial.println("Error\n\r\tDelta time to small");
    break;
  case DHTLIB_ERROR_NOTSTARTED:
    Serial.println("Error\n\r\tNot started");
    break;
  default:
    Serial.println("Unknown error");
    break;
  }
  
*/
  printEdgeTiming(_d);
}

void loop()
{

  // Launch the acquisition on the two sensors

  DHTA.acquire();
  DHTB.acquire();

  while (DHTA.acquiring())
  {
    //Serial.print(DHTA.getStatus());
    Particle.process();
  }
  Serial.print("a ");
  printEdgeTiming(&DHTA);
  Serial.print("ar ");
  printEdgeTimingResid(&DHTA);

  while (DHTB.acquiring())
  {
    //Serial.print();
    Particle.process();
  }
  Serial.print("b ");
  printEdgeTiming(&DHTB);
  Serial.print("br ");
  printEdgeTimingResid(&DHTB);

  delay(3000);
}
