#ifndef __CF_ARDU_LOG__
#define __CF_ARDU_LOG__
/* MIT License */

#include <cstdlib>

enum LogVarType
{
    LOG_UINT8 = 1,
    LOG_UINT16,
    LOG_UINT32,
    LOG_INT8,
    LOG_INT16,
    LOG_INT32,
    LOG_FLOAT,
    LOG_FP16,
};

struct LogVariable
{
    LogVarType type;
    char name[28];
    union {
        uint32_t unsignedValue;
        int32_t signedValue;
        float floatValue;
    };
};


class LogStorage
{
private:
    LogVariable *_log[255];
    unsigned int _logSize;
public:
    LogStorage() :_log(), _logSize(0) {}

    ~LogStorage() {
        for (int i=0; i<255; i++) {
            delete _log[i];
        }
    }

    unsigned int getSize() { return _logSize;}
    
    LogVariable *getVariable(unsigned int varID) {
        if (!_log[varID]) {
            return NULL;
        }
        return _log[varID];
    }

    void setVariable(unsigned int varID, LogVarType type, const char * name) {
        if (varID > 255) return;
        
        if (varID > _logSize) _logSize = varID+1;

        if (!_log[varID]) {
            _log[varID] = new LogVariable();
        }
        
        _log[varID]->type = type;
        strncpy(_log[varID]->name, name, 28);
    }
};


#endif
