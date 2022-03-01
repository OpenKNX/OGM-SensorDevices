#include "OneWireSearch.h"
#include "OneWireDS2482.h"
#ifdef COUNT_1WIRE_CHANNEL

// #ifdef ONEWIRE_TRACE_SEARCH
// onewire debug output
int OneWireSearch::searchDebug(const char* iFormat, ...)
{
    char lBuffer[256];
    uint8_t lBufferPos = gInstance * 2;
    memset(lBuffer, ' ', lBufferPos + 1);
    switch (mSearchMode)
    {
        case Family:
            lBuffer[lBufferPos] = 'F';
            break;
        case Id:
            lBuffer[lBufferPos] = 'I';
            break;
        default:
            lBuffer[lBufferPos] = 'A';
            break;
    }
    sprintf(lBuffer + lBufferPos + 1, "%02i-", gInstance + 1);
    va_list lArgs;
    va_start(lArgs, iFormat);
    int lResult = vsnprintf(lBuffer + lBufferPos + 4, 252 - lBufferPos, iFormat, lArgs);
    va_end(lArgs);
    SERIAL_DEBUG.print(lBuffer);
    return lResult;
}
// #endif

OneWireSearch::OneWireSearch(OneWireDS2482* iBM)
{
    mBM = iBM;
    gInstance = iBM->gInstance;
    mSearchState = SearchNew;
}

OneWireSearch::SearchState OneWireSearch::state()
{
    return mSearchState;
}

void OneWireSearch::newSearchAll()
{
    gInstance = mBM->gInstance;
    mSearchState = SearchNew;
    mSearchMode = All;
    mSearchFamily = 0;
    mSearchLastDeviceFlag = 0;
    mSearchLastDiscrepancy = -1;
    mSearchLastFamilyDiscrepancy = 0;
    for (uint8_t i = 0; i < 8; i++)
        mSearchResultId[i] = 0;
#if ONEWIRE_TRACE_SEARCH == detail
    searchDebug("### Init search all ###\n");
#endif
}

void OneWireSearch::newSearchFamily(uint8_t iFamily)
{
    gInstance = mBM->gInstance;
    mSearchState = SearchNew;
    mSearchMode = Family;
    mSearchFamily = iFamily;
#if ONEWIRE_TRACE_SEARCH == detail
    searchDebug("### Init search Family %02x ###\n", iFamily);
#endif
}

void OneWireSearch::newSearchNoFamily(uint8_t iFamily)
{
    gInstance = mBM->gInstance;
    mSearchState = SearchNew;
    mSearchMode = NoFamily;
    mSearchFamily = iFamily;
#if ONEWIRE_TRACE_SEARCH == detail
    searchDebug("### Init search NoFamily %02x ###\n", iFamily);
#endif
}

void OneWireSearch::newSearchForId(tIdRef iId)
{
    gInstance = mBM->gInstance;
    mSearchState = SearchNew;
    mSearchMode = Id;
    mSearchFamily = iId[0];
    mSearchLastDeviceFlag = 0;
    mSearchLastDiscrepancy = 64;
    mSearchLastFamilyDiscrepancy = 0;
    copyId(mSearchResultId, iId);
#if ONEWIRE_TRACE_SEARCH == detail
    // searchDebug("### Init search for Id %02x %02x %02x %02x %02x %02x %02x ###\n", iId[0], iId[1], iId[2], iId[3], iId[4], iId[5], iId[6]);
#endif
}

bool OneWireSearch::MatchSearchMode(uint8_t iFamily)
{
    bool lResult = false;
    switch (mSearchMode)
    {
        case All:
            lResult = true;
            break;
        case Family:
            lResult = (iFamily == mSearchFamily);
            break;
        case NoFamily:
            lResult = (iFamily != mSearchFamily);
            break;
        case Id:
            lResult = false; //todo
            break;
        default:
            break;
    }
    return lResult;
}

/****************************************
 * Due to the fact, that 1W-Devices might disappear shortly from bus
 * because of a short on the line or signal errors, we count such 
 * events and do a state change only if the event lasts for a specific
 * number of times (see cDisappearCount)
 * **************************************/
void OneWireSearch::manageSearchCounter(OneWireSearch::SearchState iState) {
    for (uint8_t i = 0; i < mBM->DeviceCount(); i++)
    {
        if (MatchSearchMode(mBM->Sensor(i)->Family()))
        {
            switch (iState)
            {
                case SearchNew:
                    mBM->Sensor(i)->incrementSearchCount();
                    break;
                case SearchFinished:
                    // All sensors, which are not found, are disconnected
                    mBM->Sensor(i)->setModeDisconnected();
                    break;
                case SearchError:
                    mBM->Sensor(i)->clearSearchCount();
                    break;
                default:
                    // do nothing
                    break;
            }
        }
    }
}

// async search, max blocking time 4 ms
// OneWireSearch::SearchState OneWireSearch::loop()
// {
//     uint8_t lStatus = 0;
//     OneWireSearch::SearchState lExitState = mSearchState;
// #ifdef DebugInfoSearch
//     uint32_t lDuration = 0;
//     mCurrDelay = millis();
// #endif
//     switch (mSearchState)
//     {
// 		case SearchNew:
// #ifdef DebugInfoSearch
// 			// DEBUG: Time measurement
// 			mDuration = millis();
//             mMaxDelay = 0;
// #endif
//             wireSearchNew();
//             mSearchState = SearchNext;
//             break;
//         case SearchNext:
//             wireSearchNext();
//             mSearchState = SearchStart;
//             mDelay = millis();
//             break;
//         case SearchStart:
//             lStatus = mBM->readStatus();//(false);
//             if (!(lStatus & DS2482_STATUS_BUSY))
//             {
//                 mSearchState = wireSearchStart(lStatus) ? SearchStep : SearchError;
//                 mDelay = millis();
//             }
//             else if (delayCheck(mDelay, 1000))
//             {
//                 mSearchState = SearchError;
//             }
//             break;
//         case SearchStep:
//             if (wireSearchStep(mSearchStep))
//             {
//                 mSearchStep += 1;
//                 // with family search we can stop as soon as an other family is found
//                 if (mSearchMode == Family && mSearchFamily != mSearchResultId[0])
//                     mSearchState = SearchFinished;
//                 if (mSearchStep == 64)
//                     mSearchState = SearchEnd;
//             }
//             else
//             {
//                 mSearchState = SearchError;
//             };
//             mDelay = millis();
//             break;
//         case SearchEnd:
//             // we do CRC check first
//             if (mSearchResultId[7] == mBM->crc8(mSearchResultId, 7)) {
//                 if (MatchSearchMode(mSearchResultId[0])) mBM->CreateOneWire(mSearchResultId);
//                 mSearchState = wireSearchEnd() ? SearchFinished : SearchNext;
//             } else {
//                 mSearchState = SearchError;
//             }
//             mDelay = millis();
//             break;
//         case SearchFinished:
//             wireSearchFinished(false);
//             mSearchState = SearchNew;
// #ifdef DebugInfoSearch
//             mMaxDelay = max(mMaxDelay, millis() - mCurrDelay);
//             lDuration = millis() - mDuration;
// 			if (abs(mDurationOld - lDuration) > 5) { 
// 				// print only if there is a big difference between cycles
//                 printDebug("(0x%X) Finished search %s (%s), took %d ms, max blocking was %d ms\n", mBM->mI2cAddress, mSearchMode == Family ? "Family" : "NoFamily", mSearchResultId, lDuration, mMaxDelay);
//                 mDurationOld = lDuration;
//             }
// #endif
//             break;
//         default:
//             wireSearchFinished(true);
//             mSearchState = SearchNew;
//             break;
//     }
// #ifdef DebugInfoSearch
//     mMaxDelay = max(mMaxDelay, millis() - mCurrDelay);
// #endif
//     return lExitState;
// }

// blocking search, takes 91 ms per sensor!!!!
// Perform a search of the 1-Wire bus
bool OneWireSearch::wireSearchBlocking()
{
    uint8_t lDirection;
    int8_t lLastZero = -1;
    bool lResult = true;
    uint8_t lCurrentByte = 0;

#if ONEWIRE_TRACE_SEARCH == detail
    // searchDebug("### start Blocking search ###\n");
#endif
    if (mSearchLastDeviceFlag)
        lResult = false;

    if (!mBM->wireReset())
        lResult = false;

    if (lResult)
    {
        mBM->waitOnBusy();

        mBM->wireWriteByte(WIRE_COMMAND_SEARCH);

        mBM->waitOnBusy();
        for (uint8_t i = 0; i < 64; i++)
        {
            int lSearchByte = i / 8;
            int lSearchBit = 1 << i % 8;

            if ((i % 4) == 0)
            {
                // callback main routine each byte found
                mBM->searchLoopCallback();
            }
            if (i < mSearchLastDiscrepancy)
                lDirection = mSearchResultId[lSearchByte] & lSearchBit;
            else
                lDirection = (i == mSearchLastDiscrepancy);

            mBM->begin();
            mBM->writeByte(DS2482_COMMAND_TRIPLET);
            mBM->writeByte(lDirection ? 0x80 : 0x00);
            mBM->end();

            uint8_t lStatus = mBM->waitOnBusy(); //(false);
            uint8_t lId = lStatus & DS2482_STATUS_SBR;
            uint8_t lCompId = lStatus & DS2482_STATUS_TSB;
            lDirection = lStatus & DS2482_STATUS_DIR;

            if (lId && lCompId)
            {
                lResult = false;
                break;
            }
            else if (!lId && !lCompId && !lDirection)
            {
                lLastZero = i;
                // check for Last discrepancy in family
                if (lLastZero < 8)
                    mSearchLastFamilyDiscrepancy = lLastZero;
            }
            lCurrentByte = mSearchResultId[lSearchByte];
            if (lDirection)
                lCurrentByte |= lSearchBit;
            else
                lCurrentByte &= ~lSearchBit;
            // in case of search for Id we just compare if
            // found bit is the same as the one in searched Id
            if (mSearchMode == Id && lCurrentByte != mSearchResultId[lSearchByte] && lSearchByte < 7)
            {
                lResult = false;
                break;
            }
            // in case of Family search, we can stop as soon as
            // an other family is found
            if (mSearchMode == Family && mSearchFamily != mSearchResultId[0])
            {
                lResult = false;
                break;
            }
            mSearchResultId[lSearchByte] = lCurrentByte;
        }
        if (lResult)
        {
            mSearchLastDiscrepancy = lLastZero;
            if (lLastZero == -1)
                mSearchLastDeviceFlag = 1;
            mBM->searchLoopCallback();
            // found id is just valid if crc8 check is ok
            if (mSearchResultId[7] != mBM->crc8(mSearchResultId, 7))
                lResult = false;
            // for (uint8_t i = 0; i < 7; i++)
            // 	eAddress[i] = mSearchResultId[i];
        }
    }
#if ONEWIRE_TRACE_SEARCH == detail
    // searchDebug("### End blocking search after %ld ms\n", millis() - lDelay);
#endif
    return lResult;
}

bool OneWireSearch::wireSearchNewDevices()
{
    if (wireSearchBlocking())
    {
        // found a device
        if (MatchSearchMode(mSearchResultId[0]))
            mBM->CreateOneWire(mSearchResultId);
        if (mSearchLastDeviceFlag)
        {
            newSearchAll();
        }
    }
    else
    {
        // if there was an error, we start over
        newSearchAll();
    }
    return true;
}
#endif
