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

#include <Console.h>
#include <AltSoftSerial.h>

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

#define PTARGETaddr 0xCF 
#define ITARGETaddr 0xE7 
#define VTARGETaddr 0xEA 

#define ISCALEaddr 0x11D
#define VSCALEaddr 0x120
#define PSCALEaddr 0x123
#define PFSCALEaddr 0x126
#define FSCALEaddr 0x129
#define TSCALEaddr 0x12C

/*==============================================================================*/
/*================================= GLOBAL VARs ================================*/
/*==============================================================================*/

#define MAX_PACKET_LEN 255
#define BAUD_RATE 38400

AltSoftSerial altSerial;
double iRes, vRes, pRes, pfRes;
double iMax = 62.5;
double vMax = 667.0;

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

boolean check_response(int response) //It'll be used only in calibration and first measurements
{
  switch(response)
  {
    case 0xAA:
//      Console.print("Acknowledge with data. Everything goes well! ");
      return(true);
      break;
    case 0xAE:
//      Console.print("Auto Reporting Header (with data). ");
      return(true);
      break;
    case 0xAD:
//      Console.print("Acknowledge without data. ");
      return(true);
      break;
    case 0xB0:
      Console.println("Negative Acknowledge (NACK).");
      break;
    case 0xBC:
      Console.println("Command not implemented.");
      break;
    case 0xBD:
      Console.println("Checksum failed.");
      break;
    case 0xBF:
      Console.println("Buffer overflow (or packet too long).");
      break;
    default:
      Console.println("TimeOut.");
  }
  return(false);
}

void select_device()
{
  unsigned char ssid = 2;
  unsigned char payload = 0xC2; //Select target device
  
  if(send_packet(1, &payload) != -1)
  {
    Console.print("Selecting device... ");
    if(check_response(rx_byte()))
    {
      Console.println("Complete!");
    }
  }
  else
  {
    Console.println("Error on the selection of the device! Packet is too large.");
  }
}

void deselect_device()
{
  unsigned char payload = 0xC0; //De-Select target device
  
  if(send_packet(1, &payload) != -1)
  {
    Console.print("Deselecting device... ");
    if(check_response(rx_byte()))
    {
      Console.println("Complete!");
    }
  }
  else
  {
    Console.println("Error on the deselection of the device! Packet is too large.");
  }
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
  else
  {
    Console.print("Packet is too long. Failed to read register with address ");
    Console.print(adrs, HEX);
    Console.println(" !");
  }

  if (emd_response[4] == 0xFF){ //If the result is negative
    reg = long(emd_response[4]); //31:24
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
  else
  {
    Console.print("Packet is too long. Failed to write ");
    Console.print(data, DEC);
    Console.print(" to register with address ");
    Console.print(adrs, HEX);
    Console.println(" !");
  }
}

/*==============================================================================*/
/*=============================== FLASH MEMORY ROUTINES ========================*/
/*==============================================================================*/

void save2FLASH() //Save to Flash Command
{
  Console.print("Saving to flash memory... ");
  writeREG(COMMANDaddr, 0xACC200);
  while(readREG(COMMANDaddr)!=0);
  Console.println("Complete!");
}

void clearFLASH() //Factory Reset
{
  Console.print("Clearing flash memory... ");
  writeREG(COMMANDaddr, 0xACC000);
  while(readREG(COMMANDaddr)!=0);
  writeREG(COMMANDaddr, 0xACC100);
  while(readREG(COMMANDaddr)!=0);
  Console.println("Complete!");
}

/*==============================================================================*/
/*================================== SCALING ROUTINES ==========================*/
/*==============================================================================*/

void scaling(long ISCALEval, long VSCALEval, long PSCALEval, int PFSCALEval)
{
  Console.println("Scaling... ");
  writeREG(ISCALEaddr, ISCALEval);
  writeREG(VSCALEaddr, VSCALEval);
  writeREG(PSCALEaddr, PSCALEval);
  writeREG(PFSCALEaddr, PFSCALEval);
  Console.println("Complete!");
  save2FLASH();
}

void computeRes()
{
  iRes = iMax/(double(readREG(ISCALEaddr)));
  vRes = vMax/(double(readREG(VSCALEaddr)));
  pRes = (vMax*iMax)/(double(readREG(PSCALEaddr)));
  pfRes = 1.0/(double(readREG(PFSCALEaddr)));
  Console.print("iResolution is: ");
  Console.println(iRes, 9);
  Console.print("vResolution is: ");
  Console.println(vRes,9);
  Console.print("pResolution is: ");
  Console.println(pRes,9);
  Console.print("pfResolution is: ");
  Console.println(pfRes,9);
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
/*================================ CALIBRATION ROUTINES ========================*/
/*==============================================================================*/

void iCAL(double value)
{
  Console.print("Calibrating current... ");
  
  long ITARGETval = long(value/iRes); //Scaling value
  
  writeREG(ITARGETaddr, ITARGETval);
  writeREG(COMMANDaddr, 0xCA0820); // HEX(CA0820) = BIN(110010100000100000100000)
  while((readREG(COMMANDaddr)>>16)!=0); //wait till 8 most significant bits are cleared, which means that the calibration is over
  Console.print("Completed ");
  if((readREG(COMMANDaddr)>>8) !=0){
    Console.print("But Failed. Try it again."); //Bits associated with any parameters (14:7 bits) that failed remain set.
  }
  else{
   Console.print("Successfully!");
  }
  Console.println();
  save2FLASH();
}

void vCAL(double value)
{
  Console.print("Calibrating voltage... ");
  
  long VTARGETval = long(value/vRes); //Scaling value
  
  writeREG(VTARGETaddr, VTARGETval);
  writeREG(COMMANDaddr, 0xCA1020); // HEX(CA1020) = BIN(110010100001000000100000)
  while((readREG(COMMANDaddr)>>16)!=0); //wait till 8 most significant bits are cleared, which means that the calibration is over
  Console.print("Completed ");
  if((readREG(COMMANDaddr)>>8) !=0){
    Console.print("But Failed. Try it again."); //Bits associated with any parameters (14:7 bits) that failed remain set.
  }
  else{
   Console.print("Successfully!");
  }
  Console.println();
  save2FLASH();
}

/*==============================================================================*/
/*================================ MEASURMENTS' ROUTINE ========================*/
/*==============================================================================*/

void print_measurements()
{
  Console.print("***********************************");
  Console.println("***********************************");
  Console.print("*                       New Measurements");
  Console.println("                             *");
  Console.print("***********************************");
  Console.println("***********************************");
  Console.println("");
  
  Console.print("RMS Voltage: ");
  Console.print(unscaleV(readREG(VRMSaddr)), 3);
  Console.println("V");  

  Console.print("RMS Current: ");
  Console.print(unscaleI(readREG(IRMSaddr)), 3);
  Console.println("A");   

  Console.print("Active Power(P): ");
  Console.print(unscaleP(readREG(WATTaddr)), 3);
  Console.println("Watt");
  
  Console.print("Reactive Power(Q): ");
  Console.print(unscaleP(readREG(VARaddr)));
  Console.println("VAR");  
  
  Console.print("Apparent Power(S): ");
  Console.print(unscaleP(readREG(VAaddr)), 3);
  Console.println("VA");
  
  Console.print("Power Factor: ");
  Console.println(unscalePF(readREG(PFaddr)), 3);  
  
  Console.print("************ ");
  Console.print(readREG(0x105));
  Console.println(" Harmonic Measurements ******************");
  
  Console.print("RMS Voltage (Harmonic): ");
  Console.print(unscaleV(readREG(VFUNDaddr)), 3);
  Console.println("V");
  
  Console.print("RMS Current (Harmonic): ");
  Console.print(unscaleI(readREG(IFUNDaddr)), 3);
  Console.println("A");
  
  Console.print("Active Power (Harmonic): ");
  Console.print(unscaleP(readREG(PFUNDaddr)), 3);
  Console.println("Watt");
  
  Console.print("Reactive Power (Harmonic): ");
  Console.print(unscaleP(readREG(QFUNDaddr)), 3);
  Console.println("VAR"); 
  
  Console.print("Apparent Power (Harmonic): ");
  Console.print(unscaleP(readREG(VAFUNDaddr)), 3);
  Console.println("VA");

  Console.print("Active Power Averaged over 30s Window: ");
  Console.print(unscaleP(readREG(PAVERAGEaddr)), 3);
  Console.println("Watt");  
  
  Console.println("");
}

/*==============================================================================*/
/*==================================== SETUP ROUTINE ===========================*/
/*==============================================================================*/

void setup() // It's executed once
{ 
  //Communication
  Bridge.begin(); //for wireless communication
  altSerial.begin(BAUD_RATE); //for the device
  Console.begin(); //for monitor
  while(!Console); //Wait till Monitor is open
  
  select_device();
  Console.print("COMMAND: ");
  Console.println(readREG(COMMANDaddr), BIN);
  
//  scaling(8000000,667000,8337500,1000);
  computeRes();
  
//  iCAL(2.786); //Set the right ITARGET measurement (without Scaling)
//  vCAL(120.0); //Set the right VTARGET measurement (without Scaling)
}

/*==============================================================================*/
/*===================================== LOOP ROUTINE ===========================*/
/*==============================================================================*/

void loop() // Runs repeatedly
{
  print_measurements();
  
  
  delay(5000);  //Gives results every 5"
}
