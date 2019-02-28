/*===========================================================================*/
/* Module :  fsl.c                                                           */
/* Version:  V1.00                                                           */
/*===========================================================================*/
/* Warranty Disclaimer                                                       */
/*                                                                           */
/* Because the Product(s) is licensed free of charge, there is no warranty   */
/* of any kind whatsoever and expressly disclaimed and excluded by Renesas,  */
/* either expressed or implied, including but not limited to those for       */
/* non-infringement of intellectual property, merchantability and/or         */
/* fitness for the particular purpose.                                       */
/* Renesas shall not have any obligation to maintain, service or provide bug */
/* fixes for the supplied Product(s) and/or the Application.                 */
/*                                                                           */
/* Each User is solely responsible for determining the appropriateness of    */
/* using the Product(s) and assumes all risks associated with its exercise   */
/* of rights under this Agreement, including, but not limited to the risks   */
/* and costs of program errors, compliance with applicable laws, damage to   */
/* or loss of data, programs or equipment, and unavailability or             */
/* interruption of operations.                                               */
/*                                                                           */
/* Limitation of Liability                                                   */
/*                                                                           */
/* In no event shall Renesas be liable to the User for any incidental,       */
/* consequential, indirect, or punitive damage (including but not limited    */
/* to lost profits) regardless of whether such liability is based on breach  */
/* of contract, tort, strict liability, breach of warranties, failure of     */
/* essential purpose or otherwise and even if advised of the possibility of  */
/* such damages. Renesas shall not be liable for any services or products    */
/* provided by third party vendors, developers or consultants identified or  */
/* referred to the User by Renesas in connection with the Product(s) and/or  */
/* the Application.                                                          */
/*                                                                           */
/*===========================================================================*/
/* History:                                                                  */
/*              V1.00: Initial version                                       */
/*                                                                           */
/*===========================================================================*/
/*                                                                           */
/* This file contains the basic tasks and mode handling.                     */
/*                                                                           */
/*===========================================================================*/

/*===========================================================================*/
/* Includes */
/*===========================================================================*/
#include "r_device.h"
#include "r_uart.h"
#include "fsl.h"

#include "fcl_cfg.h"
#include "r_fcl_types.h"
#include "r_fcl.h"
#include "target.h"
#include "fcl_descriptor.h"
#include "fcl_user.h"

/*===========================================================================*/
/* Macro Defines */
/*===========================================================================*/
#define FLMD0_PROTECTION_OFF    (0x01u)
#define FLMD0_PROTECTION_ON     (0x00u)

/*===========================================================================*/
/* Exported global variables (to be accessed by other files) */
/*===========================================================================*/
/* This array reserves the copy area in the device RAM */
#define FCL_RAM_EXECUTION_AREA_SIZE 0x8000

#if R_FCL_COMPILER == R_FCL_COMP_GHS
    #pragma ghs startdata
    #pragma ghs section bss = ".FCL_RESERVED"
    #define R_FCL_NOINIT
#elif R_FCL_COMPILER == R_FCL_COMP_IAR
    #pragma segment = "FCL_RESERVED"
    #pragma location = "FCL_RESERVED"
    #define R_FCL_NOINIT __no_init
#elif R_FCL_COMPILER == R_FCL_COMP_REC
    #pragma section r0_disp32 "FCL_RESERVED"
    #define R_FCL_NOINIT
#endif

R_FCL_NOINIT uint8_t FCL_Copy_area[FCL_RAM_EXECUTION_AREA_SIZE];

#if R_FCL_COMPILER == R_FCL_COMP_GHS
    #pragma ghs section bss = default
    #pragma ghs enddata
#elif R_FCL_COMPILER == R_FCL_COMP_IAR
    #pragma dataseg = default
#elif R_FCL_COMPILER == R_FCL_COMP_REC
    #pragma section default
#endif

/*===========================================================================*/
/* Private global variables and functions */
/*===========================================================================*/
uint8_t  WriteBuf[256];
uint32_t ReadBuf[8];

void FSL_Ctrl (void);
uint8_t FSL_UartRcvByte(void);
void FSL_CharToByte(hex_data_t* pChar, uint8_t* pByte);
void FSL_ErrorHandler(r_fcl_request_t* req);

/*===========================================================================*/
/* Functions */
/*===========================================================================*/
uint8_t FSL_Bootloader(void)
{
	r_fcl_status_t ret;
	
    #if R_FCL_COMMAND_EXECUTION_MODE == R_FCL_HANDLER_CALL_USER
        uint32_t   (* fpFct)(void);
    #endif
	
	ret = R_FCL_Init(&sampleApp_fclConfig_enu);
    if(R_FCL_OK == ret)
    {
        ret = R_FCL_CopySections();
    }
	else
	{
		R_UART_Printf("\n\r-->FCL initialization failed!\n\r");
	}
	
	if (R_FCL_OK == ret)
    {
        #if R_FCL_COMMAND_EXECUTION_MODE == R_FCL_HANDLER_CALL_USER
            fpFct = ( uint32_t (*)() )R_FCL_CalcFctAddr ( (uint32_t)(&FSL_Ctrl) );
            fpFct ();
        #else
            FSL_Ctrl ();
        #endif
    }
	
	return R_TRUE;
}

/*****************************************************************************
** Function:    FSL_Ctrl
** Description: 
** Parameter:   
** Return:      
******************************************************************************/
#if R_FCL_COMMAND_EXECUTION_MODE == R_FCL_HANDLER_CALL_USER
    #if R_FCL_COMPILER == R_FCL_COMP_GHS
        #pragma ghs section text = ".R_FCL_CODE_USR"
    #elif R_FCL_COMPILER == R_FCL_COMP_IAR
        #pragma location = "R_FCL_CODE_USR"
    #elif R_FCL_COMPILER == R_FCL_COMP_REC
        #pragma section text "R_FCL_CODE_USR"
    #endif
#endif
void FSL_Ctrl (void)
{
	uint16_t        i;
	uint8_t         IsReceiving;
	line_info_t     HexLineInfo;
	hex_line_t      HexLine;
	uint8_t         Checksum;
	
    r_fcl_request_t myRequest;
#if R_FCL_COMMAND_EXECUTION_MODE == R_FCL_HANDLER_CALL_USER
	r_fcl_request_t mySecRequest;
#endif
	
	FCLUser_Open ();
	
	/* prepare environment */
    myRequest.command_enu = R_FCL_CMD_PREPARE_ENV;
    R_FCL_Execute (&myRequest);
    #if(R_FCL_COMMAND_EXECUTION_MODE == R_FCL_HANDLER_CALL_USER)
        while(R_FCL_BUSY == myRequest.status_enu)
        {
            R_FCL_Handler();
        }
    #endif
	if(R_FCL_OK != myRequest.status_enu)
	{
		FSL_ErrorHandler(&myRequest);
	}
	
	/* Erase UserApp area(from 8 to 133 block) */
	R_UART_Printf("\n\r-->Erase UserApp area...\n\r");
	
	myRequest.command_enu = R_FCL_CMD_ERASE;
	myRequest.idx_u32     = 8;
	myRequest.cnt_u16     = (133 - 8) + 1;
    R_FCL_Execute (&myRequest);
    #if R_FCL_COMMAND_EXECUTION_MODE == R_FCL_HANDLER_CALL_USER
        while (R_FCL_BUSY == myRequest.status_enu)
        {
            R_FCL_Handler ();
        }
    #endif
	if(R_FCL_OK != myRequest.status_enu)
	{
		FSL_ErrorHandler(&myRequest);
	}
	else
	{
		R_UART_Printf("\n\r-->Erase UserApp area successfully !\n\r");
	}
	
	/* Receive Hex file via UART, and write the data to Flash */
	i = 0;
	Checksum = 0;
	HexLineInfo = LINE_RECORDMARK;
	HexLine.AddrInfo.Type = RECORD_DATA;
	HexLine.AddrInfo.Addr32 = 0x00000000;
	HexLine.AddrInfo.Addr32BU = HexLine.AddrInfo.Addr32;
	IsReceiving = R_TRUE;
	memset(&WriteBuf[0], 0x00, sizeof(WriteBuf));
	R_UART_Printf("\n\r-->Please send Hex file!\n\r");
	
	while(IsReceiving == R_TRUE)
	{
		switch(HexLineInfo)
		{
			case LINE_RECORDMARK:
			    /* Receive the first Hex line data, it must be record mark ':', discard 
			       the record mark and repare to receive the next coming data; otherwise,
			       the received file is not Hex file, exits UART receiving. */
			    if(':' == R_RLIN30_UartRcvData())
				{
					/* The following data is record length */
					HexLineInfo = LINE_RECORDLENGTH;
				}
				else
				{
					/* Exits UART receiving */
					IsReceiving = R_FALSE;
					R_UART_Printf("\n\rExit Flash Self-programming, Please send Hex file!\n\r");					
				}
			    break;
					
			case LINE_RECORDLENGTH:
			    /* 1Byte record length data received */	
			    HexLine.Length = FSL_UartRcvByte();
				/* Calculate the checksum */
				Checksum += HexLine.Length;
				
				/* The data following record length is load offset */
				HexLineInfo = LINE_LOADOFFSET;
			    break;
				
			case LINE_LOADOFFSET:
			    /* Receive 2Bytes load offset data and calculate the checksum */	
			    HexLine.AddrInfo.Offset.U16_H = FSL_UartRcvByte();
				Checksum += HexLine.AddrInfo.Offset.U16_H;
				HexLine.AddrInfo.Offset.U16_L = FSL_UartRcvByte();
				Checksum += HexLine.AddrInfo.Offset.U16_L;
				
				/* Calculate Offset */
				HexLine.AddrInfo.Offset.U16 = (uint16_t)HexLine.AddrInfo.Offset.U16_H << 8;
				HexLine.AddrInfo.Offset.U16 += HexLine.AddrInfo.Offset.U16_L;
				
				/* Calculate the absolute address */
				if(RECORD_EXTENDED_SEGMENT_ADDRESS == HexLine.AddrInfo.Type)
				{
					/* Extend segment address */
					HexLine.AddrInfo.Addr32 = (uint32_t)HexLine.AddrInfo.Extend.U16 << 4;
					HexLine.AddrInfo.Addr32 += HexLine.AddrInfo.Offset.U16;
				}
				else if(RECORD_EXTENDED_LINEAR_ADDRESS == HexLine.AddrInfo.Type)
				{
					/* Extend linear address */
					HexLine.AddrInfo.Addr32 = (uint32_t)HexLine.AddrInfo.ExtdLinear.U16 << 16;
					HexLine.AddrInfo.Addr32 += HexLine.AddrInfo.Offset.U16;
				}
				
				/* If the hex line is not data line, but an address information, a new start address
				   is received, the old address shuld be backed up for */
				if(RECORD_DATA != HexLine.RecordType)
				{
					HexLine.AddrInfo.Addr32BU =  HexLine.AddrInfo.Addr32;
				}
				
				/* The data following load offset is record type */
				HexLineInfo = LINE_RECORDTYPE;
			    break;
				
			case LINE_RECORDTYPE:
			    /* 1Byte record type data received */
			    HexLine.RecordType = (record_t)FSL_UartRcvByte();
				/* Calculate the checksum */
				Checksum += HexLine.RecordType;
				
				/* If the record type is 01, Hex file transmiting is going to finish */
				if(RECORD_END_OF_FILE == HexLine.RecordType)
				{
					IsReceiving = R_FALSE;
					R_UART_Printf("\n\rEnd of Hex file.\n\r");
					R_UART_Printf("\n\rSuccessfully Update UserApp code!\n\r");
					break;
				}
				else
				{
					/* The data following record type is hex data */
					HexLineInfo = LINE_DATA;
				}
			    break;
			
		    /* Receive data... */
			case LINE_DATA:
			    /* The received hex data are stored in HexLine.Data and WriteBuf, while WriteBuf
				   is full(256bytes), write the data in WriteBuf to Flash, and then clear WriteBuf
				   for the new data. */
			    if(RECORD_DATA == HexLine.RecordType)
				{   
					/* The starting address of the new line is the continuous address of the previous
					   line, the data of new line are stored in previous WriteBuf */
					if (HexLine.AddrInfo.Addr32BU == HexLine.AddrInfo.Addr32)
					{
						/* Received a line hex data */
						for (i = 0; i < HexLine.Length; i++)
				    	{
							/* Received data are stored in HexLine.Data */
							HexLine.Data[i] = FSL_UartRcvByte();
				            Checksum += HexLine.Data[i];
							
							/* Write data in HexLine.Data to WriteBuf, the starting WriteBuf has to 256bytes
							   alignment */
							WriteBuf[(HexLine.AddrInfo.Addr32 & 0xFF)] = HexLine.Data[i];
							HexLine.AddrInfo.Addr32++;
							
							/* WriteBuf is full, write the data in WirteBuf to Flash */
							if ((HexLine.AddrInfo.Addr32 & 0xFF) == 0)
							{
								myRequest.command_enu = R_FCL_CMD_WRITE;
								myRequest.bufferAdd_u32 = (uint32_t)&WriteBuf[0];
								myRequest.idx_u32       = HexLine.AddrInfo.Addr32 - 0x100;
								myRequest.cnt_u16       = 1;
								R_FCL_Execute (&myRequest);
                            	#if R_FCL_COMMAND_EXECUTION_MODE == R_FCL_HANDLER_CALL_USER
                            	    while (R_FCL_BUSY == myRequest.status_enu)
                            	    {
                                	    R_FCL_Handler ();
                            	    }
                            	#endif
								
								if(R_FCL_OK != myRequest.status_enu)
	                            {
		                            FSL_ErrorHandler(&myRequest);
	                            }
								
								/* Clear WriteBuf for new data storage */
								memset(&WriteBuf[0], 0x00, sizeof(WriteBuf));
							}
						}
						
						/* Complete to receive a line data, backup the address */
						HexLine.AddrInfo.Addr32BU =  HexLine.AddrInfo.Addr32;
					}
					/* The starting address of the new line is discontinuous from the previous line,
					   recalculate the address, if the gap from previous address is greater than the
					   remaining WriteBuf, firstly, write the data which exist in WrtieBuf to Flash, 
					   and then, receive the new data and stored it into a new WriteBuf */
					else if((256 - (HexLine.AddrInfo.Addr32BU & 0xFF)) >= (HexLine.AddrInfo.Addr32 - HexLine.AddrInfo.Addr32BU))
					{
						myRequest.command_enu = R_FCL_CMD_WRITE;
						myRequest.bufferAdd_u32 = (uint32_t)&WriteBuf[0];
						myRequest.idx_u32       = (HexLine.AddrInfo.Addr32BU & 0xFFFFFF00);
						myRequest.cnt_u16       = 1;
						R_FCL_Execute (&myRequest);
                        #if R_FCL_COMMAND_EXECUTION_MODE == R_FCL_HANDLER_CALL_USER
                            while (R_FCL_BUSY == myRequest.status_enu)
                            {
                           	    R_FCL_Handler ();
                            }
                        #endif
								
						if(R_FCL_OK != myRequest.status_enu)
	                    {
		                    FSL_ErrorHandler(&myRequest);
	                    }
								
						/* Clear WriteBuf for new data storage */
						memset(&WriteBuf[0], 0x00, sizeof(WriteBuf));
						/* Back up Addr32 */
						HexLine.AddrInfo.Addr32BU = HexLine.AddrInfo.Addr32;
						
						/* Continue receiving new data */
			        	for (i = 0; i < HexLine.Length; i++)
				    	{
			            	HexLine.Data[i] = FSL_UartRcvByte();
				            Checksum += HexLine.Data[i];
					    
							WriteBuf[(HexLine.AddrInfo.Addr32 & 0xFF)] = HexLine.Data[i];
							HexLine.AddrInfo.Addr32++;
						}
							
						/* Complete to receive a line data, backup the address */
						HexLine.AddrInfo.Addr32BU =  HexLine.AddrInfo.Addr32;
					}
				}
				/* The received data is extended segment address data */
				else if(RECORD_EXTENDED_SEGMENT_ADDRESS == HexLine.RecordType)
				{
					/* If the record type is 02, Receive 2Bytes extend address data */
			        HexLine.AddrInfo.Extend.U16_H = FSL_UartRcvByte();
				    Checksum += HexLine.AddrInfo.Extend.U16_H;
				    HexLine.AddrInfo.Extend.U16_L = FSL_UartRcvByte();
				    Checksum += HexLine.AddrInfo.Extend.U16_L;
					
					/* Calculate Extend */
					HexLine.AddrInfo.Extend.U16 = (uint16_t)HexLine.AddrInfo.Extend.U16_H << 8;
					HexLine.AddrInfo.Extend.U16 += HexLine.AddrInfo.Extend.U16_L;
					
					/* Address type is the extended segment address */
					HexLine.AddrInfo.Type = RECORD_EXTENDED_SEGMENT_ADDRESS;
				}
				/* The received data is extended linear address data */
				else if(RECORD_EXTENDED_LINEAR_ADDRESS == HexLine.RecordType)
				{
					/* If the record type is 04, Receive 2Bytes extend linear address data */
			        HexLine.AddrInfo.ExtdLinear.U16_H = FSL_UartRcvByte();
				    Checksum += HexLine.AddrInfo.ExtdLinear.U16_H;
				    HexLine.AddrInfo.ExtdLinear.U16_L = FSL_UartRcvByte();
				    Checksum += HexLine.AddrInfo.ExtdLinear.U16_L;
					
					/* Calculate Extend */
					HexLine.AddrInfo.ExtdLinear.U16 = (uint16_t)HexLine.AddrInfo.ExtdLinear.U16_H << 8;
					HexLine.AddrInfo.ExtdLinear.U16 += HexLine.AddrInfo.ExtdLinear.U16_L;
					
					/* Address type is the extended segment address */
					HexLine.AddrInfo.Type = RECORD_EXTENDED_LINEAR_ADDRESS;
				}
				
				/* The data following hex data is checksum */
				HexLineInfo = LINE_CHECKSUM;
			    break;
				
			case LINE_CHECKSUM:
			    /* 1Byte checksum data received */
			    HexLine.Checksum = FSL_UartRcvByte();
				
				/* Verify the checksum */
				if((0x100 - Checksum) != HexLine.Checksum)
				{
					IsReceiving = R_FALSE;
					R_UART_Printf("\n\rFail to receive Hex file!\n\r");
					break;
				}
				
				/* Prepare to receive the data of hex line end */
				HexLineInfo = LINE_END;
			    break;
				
			case LINE_END:
			    /* Receive the end of line data '0xd'&'0x0a', discard it */
				R_RLIN30_UartRcvData();
				R_RLIN30_UartRcvData();
				
				/* Prepare to receive the next Hex line data  */
				i = 0;
				Checksum = 0;
				HexLineInfo = LINE_RECORDMARK;
			    break;
				
			default:
			    IsReceiving = R_FALSE;
			    break;
		}
	}
		
	FCLUser_Close ();
}

/*****************************************************************************
** Function:    FSL_UartRcvByte
** Description: Uart receives two ASCII data, converts and combines these two
**              ASCII data to a byte data
** Parameter:   
** Return:      Byte data;     
******************************************************************************/
#if   R_FCL_COMPILER == R_FCL_COMP_GHS
  #pragma ghs section text =".R_FCL_CODE_USR"
#elif R_FCL_COMPILER == R_FCL_COMP_IAR
  #pragma location = "R_FCL_CODE_USR"
#elif R_FCL_COMPILER == R_FCL_COMP_REC
  #pragma section text "R_FCL_CODE_USR"
#endif

uint8_t FSL_UartRcvByte(void)
{
	hex_data_t tmp;
	
	tmp.CharH = R_RLIN30_UartRcvData();
	tmp.CharL = R_RLIN30_UartRcvData();
	FSL_CharToByte(&tmp, &tmp.Byte);
	
	return tmp.Byte;
}


/*****************************************************************************
** Function:    CharToByte
** Description: 
** Parameter:   
** Return:      
******************************************************************************/
#if   R_FCL_COMPILER == R_FCL_COMP_GHS
  #pragma ghs section text =".R_FCL_CODE_USR"
#elif R_FCL_COMPILER == R_FCL_COMP_IAR
  #pragma location = "R_FCL_CODE_USR"
#elif R_FCL_COMPILER == R_FCL_COMP_REC
  #pragma section text "R_FCL_CODE_USR"
#endif

void FSL_CharToByte(hex_data_t* pChar, uint8_t* pByte)
{
	char_t high, low;
	high = pChar->CharH;
	low = pChar->CharL;
	
	if((low >= '0') && (low <= '9'))
	{
		low = low - '0';
	}
	else if((low >= 'a') && (low <= 'f'))
	{
		low = low - 'a' + 0xa;
	}
	else if((low >= 'A') && (low <= 'F'))
	{
		low = low - 'A' + 0xa;
	}
	
	if((high >= '0') && (high <= '9'))
	{
		high = high - '0';
	}
	else if((high >= 'a') && (high <= 'f'))
	{
		high = high - 'a' + 0xa;
	}
	else if((high >= 'A') && (high <= 'F'))
	{
		high = high - 'A' + 0xa;
	}
	
	*pByte = (uint8_t)high * 16 + low;
}

/*****************************************************************************
** Function:    FSL_ErrorHandler
** Description: 
** Parameter:   
** Return:      
******************************************************************************/
#if   R_FCL_COMPILER == R_FCL_COMP_GHS
  #pragma ghs section text =".R_FCL_CODE_USR"
#elif R_FCL_COMPILER == R_FCL_COMP_IAR
  #pragma location = "R_FCL_CODE_USR"
#elif R_FCL_COMPILER == R_FCL_COMP_REC
  #pragma section text "R_FCL_CODE_USR"
#endif
void FSL_ErrorHandler(r_fcl_request_t* req)
{
	switch(req->command_enu)
	{
		case R_FCL_CMD_PREPARE_ENV:
		    R_UART_Printf("\n\r-->Failed to execute R_FCL_CMD_PREPARE_ENV! Error = %d", req->status_enu);
		    break;
		case R_FCL_CMD_ERASE:
		    R_UART_Printf("\n\r-->Failed to execute R_FCL_CMD_ERASE! Error = %d", req->status_enu);
		    break;
		case R_FCL_CMD_WRITE:
		    R_UART_Printf("\n\r-->Failed to execute R_FCL_CMD_WRITE! Error = %d", req->status_enu);
		    break;
		case R_FCL_CMD_SET_LOCKBIT:
		case R_FCL_CMD_GET_LOCKBIT:
		case R_FCL_CMD_ENABLE_LOCKBITS:
		case R_FCL_CMD_DISABLE_LOCKBITS:
		case R_FCL_CMD_SET_OTP:
		case R_FCL_CMD_GET_OTP:
		case R_FCL_CMD_SET_OPB:
		case R_FCL_CMD_GET_OPB:
		case R_FCL_CMD_SET_ID:
		case R_FCL_CMD_GET_ID:
		case R_FCL_CMD_SET_READ_PROTECT_FLAG:
		case R_FCL_CMD_GET_READ_PROTECT_FLAG:
		case R_FCL_CMD_SET_WRITE_PROTECT_FLAG:
		case R_FCL_CMD_GET_WRITE_PROTECT_FLAG:
		case R_FCL_CMD_SET_ERASE_PROTECT_FLAG:
		case R_FCL_CMD_GET_ERASE_PROTECT_FLAG:
		case R_FCL_CMD_SET_SERIAL_PROG_DISABLED:
		case R_FCL_CMD_GET_SERIAL_PROG_DISABLED:
		case R_FCL_CMD_SET_SERIAL_ID_ENABLED:
		case R_FCL_CMD_GET_SERIAL_ID_ENABLED:
		case R_FCL_CMD_SET_RESET_VECTOR:
		case R_FCL_CMD_GET_RESET_VECTOR:
		case R_FCL_CMD_GET_BLOCK_CNT:
		case R_FCL_CMD_GET_BLOCK_END_ADDR:
		case R_FCL_CMD_GET_DEVICE_NAME:
		default:
		    R_UART_Printf("\n\r-->Failed to execute R_FCL_COMMAND!\n\r");
		    break;
	}
	
	while(1) {}
}