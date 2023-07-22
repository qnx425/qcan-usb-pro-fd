#include <stm32h7xx_hal.h>
#include <usbd_def.h>
#include <usbd_core.h>

#define USB_WITHOUT_ISR 1
PCD_HandleTypeDef hpcd_usb;
void Error_Handler(void);

USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status);

#if ( USB_WITHOUT_ISR == 0 )
void OTG_HS_IRQHandler(void)
{
  HAL_PCD_IRQHandler( &hpcd_usb );
}

void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler( &hpcd_usb );
}
#endif

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetupStage((USBD_HandleTypeDef*)hpcd->pData, (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataOutStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataInStage((USBD_HandleTypeDef*)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SOF((USBD_HandleTypeDef*)hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{ 
  USBD_SpeedTypeDef speed = USBD_SPEED_FULL;

  if ( hpcd->Init.speed == PCD_SPEED_HIGH)
  {
    speed = USBD_SPEED_HIGH;
  }
  else if ( hpcd->Init.speed == PCD_SPEED_FULL)
  {
    speed = USBD_SPEED_FULL;
  }
  else
  {
    Error_Handler();
  }
  
  USBD_LL_SetSpeed((USBD_HandleTypeDef*)hpcd->pData, speed);

  USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  /* Inform USB library that core enters in suspend Mode. */
  USBD_LL_Suspend((USBD_HandleTypeDef*)hpcd->pData);
  __HAL_PCD_GATE_PHYCLOCK(hpcd);
  /* Enter in STOP mode. */
  if (hpcd->Init.low_power_enable)
  {
    /* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register. */
    SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
  }
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_Resume((USBD_HandleTypeDef*)hpcd->pData);
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_IsoINIncomplete((USBD_HandleTypeDef*)hpcd->pData, epnum);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected((USBD_HandleTypeDef*)hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected((USBD_HandleTypeDef*)hpcd->pData);
}

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
  if (pdev->id == DEVICE_FS) {
  hpcd_usb.pData = pdev;
  pdev->pData = &hpcd_usb;

  /* Init USB Ip. */
  hpcd_usb.Instance = USB_OTG_FS;
  hpcd_usb.Init.dev_endpoints = 9;
  hpcd_usb.Init.speed = PCD_SPEED_FULL;
  hpcd_usb.Init.dma_enable = DISABLE;
  hpcd_usb.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_usb.Init.Sof_enable = DISABLE;
  hpcd_usb.Init.low_power_enable = DISABLE;
  hpcd_usb.Init.lpm_enable = DISABLE;
  hpcd_usb.Init.battery_charging_enable = DISABLE;
  hpcd_usb.Init.vbus_sensing_enable = DISABLE;
  hpcd_usb.Init.use_dedicated_ep1 = DISABLE;

  if( HAL_PCD_Init( &hpcd_usb ) != HAL_OK )
  {
    Error_Handler( );
  }
  
  HAL_PCDEx_SetRxFiFo(&hpcd_usb, 0x80);
  HAL_PCDEx_SetTxFiFo(&hpcd_usb, 0, 0x40);
  HAL_PCDEx_SetTxFiFo(&hpcd_usb, 1, 0x80);
  HAL_PCDEx_SetTxFiFo(&hpcd_usb, 2, 0x80);
  HAL_PCDEx_SetTxFiFo(&hpcd_usb, 3, 0x80);
  }
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_DeInit(pdev->pData);

  usb_status =  USBD_Get_USB_Status(hal_status);
  
  return usb_status; 
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
 
  hal_status = HAL_PCD_Start(pdev->pData);
  
  usb_status =  USBD_Get_USB_Status(hal_status);     
  
  return usb_status;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_Stop(pdev->pData);
  
  usb_status =  USBD_Get_USB_Status(hal_status);
 
  return usb_status;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);

  usb_status =  USBD_Get_USB_Status(hal_status);
  
  return usb_status;
}


USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  
  hal_status = HAL_PCD_EP_Close(pdev->pData, ep_addr);
  
  usb_status =  USBD_Get_USB_Status(hal_status);    
 
  return usb_status;  
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  
  hal_status = HAL_PCD_EP_Flush(pdev->pData, ep_addr);
  
  usb_status =  USBD_Get_USB_Status(hal_status);   
  
  return usb_status;  
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  
  hal_status = HAL_PCD_EP_SetStall(pdev->pData, ep_addr);

  usb_status =  USBD_Get_USB_Status(hal_status);
  
  return usb_status;  
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  
  hal_status = HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
    
  usb_status =  USBD_Get_USB_Status(hal_status);  
  
  return usb_status; 
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;
  
  if((ep_addr & 0x80) == 0x80)
  {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall; 
  }
  else
  {
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall; 
  }
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;
  
  hal_status = HAL_PCD_SetAddress(pdev->pData, dev_addr);
  
  usb_status =  USBD_Get_USB_Status(hal_status);
  
  return usb_status;  
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  
  usb_status =  USBD_Get_USB_Status(hal_status); 
  
  return usb_status;    
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  USBD_StatusTypeDef usb_status = USBD_OK;

  hal_status = HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
  
  usb_status =  USBD_Get_USB_Status(hal_status);   
  
  return usb_status; 
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef*) pdev->pData, ep_addr);
}

void USBD_LL_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status)
{
  USBD_StatusTypeDef usb_status = USBD_OK;

  switch (hal_status)
  {
    case HAL_OK :
      usb_status = USBD_OK;
    break;
    case HAL_ERROR :
      usb_status = USBD_FAIL;
    break;
    case HAL_BUSY :
      usb_status = USBD_BUSY;
    break;
    case HAL_TIMEOUT :
      usb_status = USBD_FAIL;
    break;
    default :
      usb_status = USBD_FAIL;
    break;
  }
  return usb_status;
}
