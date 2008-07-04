#define COMMANDREG 0x01
#define COMMLENREG 0x02
#define ERRORREG 0x06
#define FIFODATAREG 0x09
#define FIFOLEVELREG 0x0A
#define CONTROLREG 0x0C
#define BITFRAMINGREG 0x0D
#define TXMODEREG 0x12
#define RXMODEREG 0x13
#define TXCONTROLREG 0x14
#define TXAUTOREG 0x15
#define RXTRESHOLDREG 0x18
#define DEMODREG 0x19
#define GSNOFFREG 0x23
#define MODWIDTHREG 0x24
#define TXBITPHASEREG 0x25
#define RFCFGREG 0x26
#define GSNONREG 0x27
#define CWGSPREG 0x28
#define MODGSPREG 0x29

void SPIinit(void);
void SR(uint8_t addr, uint8_t dta);
uint8_t GR(uint8_t addr);
extern void ActivateReader(void);
extern void Authenticate(uint8_t dta[]);
extern void RequestMifare(uint8_t *dta);
