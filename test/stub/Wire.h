#ifndef GS_STUB_WIRE_H
#define GS_STUB_WIRE_H
class TwoWireStub { public: void setSDA(int) {} void setSCL(int) {} void begin() {} void setClock(long) {} };
extern TwoWireStub Wire;
#endif
