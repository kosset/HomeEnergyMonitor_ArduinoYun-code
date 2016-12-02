/*
Home Energy Monitor using Arduino Yun

Seretakis K, Setzas K
Aristotle University of Thessaloniki
Smart Grids
Supervisor: George Andreou

v1.0 July 2015
*/

/*==============================================================================*/
/*================================= LIBRARIES ==================================*/
/*==============================================================================*/

#include <AltSoftSerial.h> // AltSoftSerial for Yun always uses these pins: Tx=5, Rx=13
#include <Process.h> // for processing in Linino

/*==============================================================================*/
/*================================= REG's ADDRESSES ============================*/
/*==============================================================================*/

#define COMMANDaddr 0x00

#define VAaddr 0x15
#define VARaddr 0x18
#define VRMSaddr 0x1B
#define IRMSaddr 0x1E
#define WATTaddr 0x21
#define PAVERAGEaddr 0x24
#define PFaddr 0x27

#define HARMaddr 0x105
#define VFUNDaddr 0x33
#define IFUNDaddr 0x36
#define PFUNDaddr 0x39
#define QFUNDaddr 0x3C
#define VAFUNDaddr 0x3F
//#define VHARMaddr 0x42
//#define IHARMaddr 0x45
//#define PHARMaddr 0x48
//#define QHARMaddr 0x4B
//#define VAHARMaddr 0x4E
#define ILOWaddr 0x57
#define IHIGHaddr 0x5A
#define VLOWaddr 0x60
#define VHIGHaddr 0x63

#define ISCALEaddr 0x11D
#define VSCALEaddr 0x120
#define PSCALEaddr 0x123
#define PFSCALEaddr 0x126

/*==============================================================================*/
/*================================= GLOBAL VARs ================================*/
/*==============================================================================*/

#define MAX_PACKET_LEN 255
#define BAUD_RATE 38400

AltSoftSerial altSerial;

double iRes, vRes, pRes, pfRes;
double iMax = 62.5;
double vMax = 667;

/*==============================================================================*/
/*============================== GENERAL UART FUNCTIONS ========================*/
/*==============================================================================*/

unsigned char rx_byte()
{
  //timeout = ~20ms (Because bytes * bits_per_character / bits_per_second, so 50*(8+1+1)/38400=13,02 ms)
  unsigned char bfr[1];
  altSerial.setTimeout(20); 
  altSerial.readBytes(bfr,1);
  return(bfr[0]);  
}

void tx_byte(unsigned char data)
{
  altSerial.write(data);
}

boolean check_response(int response)
{
   if((response == 0xAA) || (response == 0xAE) || (response == 0xAD))
   {
     return(true);
   }
   return(false);
}

unsigned char get_checksum(unsigned char cnt, unsigned char *data)
{
  int chksum = 0;
  unsigned char idx;
  
  for(idx = 0; idx < (cnt - 1); idx++)
  {
    chksum += data[idx];
  }
  
  chksum = (~(chksum%0x100) + 1);
  
  return((unsigned char) (0x00FF & chksum));
}

int send_packet(unsigned char byte_cnt, unsigned char *payload)
{
  unsigned char idx;
  unsigned char packet_length = (byte_cnt + 3); //+3 due to header, byte_cnt, and checksum
  unsigned char packet[MAX_PACKET_LEN];
  
  //Check packet length
  if(packet_length > MAX_PACKET_LEN)
  {
    return(-1);
  }
  
  packet[0] = 0xAA; //Header
  packet[1] = packet_length; 
  
  //stuff payload into packet
  for(idx = 0; idx < byte_cnt; idx++)
  {
    packet[idx + 2] = payload[idx];
  }
  
  //get checksum
  packet[idx+2] = get_checksum(packet_length, packet);
  
  //send it down the pipe
  for(idx = 0; idx < packet_length; idx++)
  {
    tx_byte(packet[idx]);
  }
  
  return 1;
}

void select_device()
{
  unsigned char payload = 0xC2; //Select target device
  send_packet(1, &payload);
  check_response(rx_byte());
}

void deselect_device()
{
  unsigned char payload = 0xC0; //De-Select target device  
  send_packet(1, &payload);
  check_response(rx_byte());
}

/* ======= Read/Write Commands ======== */
long readREG(int adrs)
{
  long reg = 0;
  static unsigned char emd_response[6];
  unsigned char byte_cnt = 0;
  unsigned char payload[4];
  
  //flush response buffer from last time
  for(byte_cnt = 0; byte_cnt < 6; byte_cnt++)
  {
    emd_response[byte_cnt] = 0;
  }
  
  byte_cnt = 0; //reset byte_cnt
  
  payload[byte_cnt++] = 0xA3; //Set target address bits [15:0].
  payload[byte_cnt++] = ((unsigned char) (0x00ff&adrs)); //LSB of address
  payload[byte_cnt++] = ((unsigned char) (0x00ff&(adrs >> 8))); //MSB of address 
  
  payload[byte_cnt++] = 0xE3; //Read 3 bytes

  if(send_packet(byte_cnt, payload) != -1)
  {
    for(byte_cnt = 0; byte_cnt < 6; byte_cnt++)
    {
      emd_response[byte_cnt] = rx_byte();
    }
    check_response(emd_response[0]); //Checking the response of the slave (device)
  }
  
  if ((emd_response[4]>>7) == 1){ //If the result is negative
    reg = long(0xFF); //31:24
    reg = reg << 8;
  }
  reg = reg + long(emd_response[4]); //23:16
  reg = reg << 8;
  reg = reg + long(emd_response[3]); //15:8
  reg = reg << 8;
  reg = reg + long(emd_response[2]); //7:0
    
  return(reg);
}

void writeREG(int adrs, long data)
{
  unsigned char byte_cnt = 0;
  unsigned char payload[7];
  
  payload[byte_cnt++] = 0xA3;
  payload[byte_cnt++] = ((unsigned char) (0x00FF & adrs)); //LSB of address
  payload[byte_cnt++] = ((unsigned char) (0x00FF & (adrs >> 8))); //MSB of address
  
  payload[byte_cnt++] = 0xD3; //Write 3 bytes
  payload[byte_cnt++] = ((unsigned char) (0x000000FF & data)); //7:0
  payload[byte_cnt++] = ((unsigned char) (0x000000FF & (data >> 8))); //15:8
  payload[byte_cnt++] = ((unsigned char) (0x000000FF & (data >> 16))); //23:16
  
  if(send_packet(byte_cnt, payload) != -1)
  {
    check_response(rx_byte());
  }
}

/*==============================================================================*/
/*=============================== FLASH MEMORY ROUTINES ========================*/
/*==============================================================================*/

void save2FLASH() //Save to Flash Command
{
  writeREG(COMMANDaddr, 0xACC200);
  while(readREG(COMMANDaddr)!=0);
}

void clearFLASH() //Factory Reset
{
  writeREG(COMMANDaddr, 0xACC000);
  while(readREG(COMMANDaddr)!=0);
  writeREG(COMMANDaddr, 0xACC100);
  while(readREG(COMMANDaddr)!=0);
}

/*==============================================================================*/
/*=================================== SCALING ROUTINES ==========================*/
/*==============================================================================*/

void computeRes()
{
  iRes = iMax/(double(readREG(ISCALEaddr)));
  vRes = vMax/(double(readREG(VSCALEaddr)));
  pRes = (vMax*iMax)/(double(readREG(PSCALEaddr)));
  pfRes = 1.0/(double(readREG(PFSCALEaddr)));
}

double unscaleI(long value)
{
  return(iRes * (double(value)));
}

double unscaleV(long value)
{
  return(vRes * (double(value)));
}

double unscaleP(long value)
{
  return(pRes * (double(value)));
}

double unscalePF(long value)
{
  return(pfRes * (double(value)));
}


/*==============================================================================*/
/*================================ MEASURMENTS' ROUTINE ========================*/
/*==============================================================================*/

void save2DB()
{
  char buff[32];
  String value;
  
  Process p;
  p.begin("/mnt/sda1/arduino/www/HEM/insertIntoDB.php");
  
  //Vrms
  value = dtostrf(unscaleV(readREG(VRMSaddr)), 6, 3, buff);
  p.addParameter(value);
 
  //Irms
  value = dtostrf(unscaleI(readREG(IRMSaddr)), 6, 3, buff);
  p.addParameter(value);
  
  //P
  value = dtostrf(unscaleP(readREG(WATTaddr)), 6, 3, buff);
  p.addParameter(value);
  
  //Q
  value = dtostrf(unscaleP(readREG(VARaddr)), 6, 3, buff);
  p.addParameter(value);
  
  //S
  value = dtostrf(unscaleP(readREG(VAaddr)), 6, 3, buff);
  p.addParameter(value);
  
  //pf
  value = dtostrf(unscalePF(readREG(PFaddr)), 6, 3, buff);
  p.addParameter(value);  
  
  //VrmsHarmonic
  value = dtostrf(unscaleV(readREG(VFUNDaddr)), 6, 3, buff);
  p.addParameter(value);
 
  //IrmsHarmonic
  value = dtostrf(unscaleI(readREG(IFUNDaddr)), 6, 3, buff);
  p.addParameter(value);
  
  //Pharmonic
  value = dtostrf(unscaleP(readREG(PFUNDaddr)), 6, 3, buff);
  p.addParameter(value);
  
  //Qharmonic
  value = dtostrf(unscaleP(readREG(QFUNDaddr)), 6, 3, buff);
  p.addParameter(value);
  
  //Sharmonic
  value = dtostrf(unscaleP(readREG(VAFUNDaddr)), 6, 3, buff);
  p.addParameter(value);
  
  //VrmsHigh
  value = dtostrf(unscaleV(readREG(VHIGHaddr)), 6, 3, buff);
  p.addParameter(value);
  
  //VrmsLow
  value = dtostrf(unscaleV(readREG(VLOWaddr)), 6, 3, buff);
  p.addParameter(value);
  
  //IrmsHigh
  value = dtostrf(unscaleI(readREG(IHIGHaddr)), 6, 3, buff);
  p.addParameter(value);
  
  //IrmsLow
  value = dtostrf(unscaleI(readREG(ILOWaddr)), 6, 3, buff);
  p.addParameter(value);
  
  //Paverage
  value = dtostrf(unscaleP(readREG(PAVERAGEaddr)), 6, 3, buff);
  p.addParameter(value);
  
  p.run();
}

/*==============================================================================*/
/*==================================== SETUP ROUTINE ===========================*/
/*==============================================================================*/

void setup() // It's executed once
{ 
  altSerial.begin(BAUD_RATE); //Serial Initialization for the device
  Bridge.begin();  //Initialize Bridge

  select_device();
  
  writeREG(HARMaddr, 3); //Select harmonic
  computeRes();

}

/*==============================================================================*/
/*===================================== LOOP ROUTINE ===========================*/
/*==============================================================================*/

void loop() // Runs repeatedly
{
  save2DB();
  
  delay(5000);  //Gives results every 5"
}
