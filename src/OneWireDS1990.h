#pragma once

#include "OneWire.h"
#include <inttypes.h>
#ifdef WIREMODULE
    #include "OneWireDS2482.h"

class OneWireDS1990 : public OneWire
{
  public:
    OneWireDS1990(tIdRef iId);

    void loop() override;

    bool getValue(uint8_t &eValue, uint8_t iModelFunction) override;

  private:
};
#endif
