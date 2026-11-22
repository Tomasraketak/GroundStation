#ifndef GS_STUB_SPI_H
#define GS_STUB_SPI_H
class SPIClassStub { public: void setRX(int) {} void setTX(int) {} void setSCK(int) {} void begin() {} };
extern SPIClassStub SPI1;
#endif
