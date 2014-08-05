/*This code is based off of Andy Kirkham's
*work making MODDMA.
*http://mbed.org/users/AjK/code/MODDMA/
*/
#include "mbed.h"
#include "MODDMA.h"
#include "string.h"
#include "EthernetInterface.h"

#define NUM_SAMPLES 1024
#define BYTES_PER_SAMPLE sizeof(uint16_t) / sizeof(char)
#define NUM_CHANNELS 4
#define SAMPLE_BUFF_LEN     NUM_SAMPLES*NUM_CHANNELS
#define SAMPLE_BUFF_BYTES   SAMPLE_BUFF_LEN * BYTES_PER_SAMPLE

DigitalOut led1(LED1);
DigitalOut led2(LED2);

MODDMA dma;
const int BROADCAST_PORT = 58083;

// ISR set's this when transfer complete.
bool dmaTransferComplete = false;

// Function prototypes for IRQ callbacks.
// See definitions following main() below.
void TC0_callback(void);
void ERR0_callback(void);

int main() {
    //Set up the ethernet to broadcast
    EthernetInterface eth;
    eth.init(); //Use DHCP
    eth.connect();
    
    UDPSocket sock;
    sock.init();
    sock.set_broadcasting();
    
    Endpoint broadcast;
    broadcast.set_address("255.255.255.255", BROADCAST_PORT);
    
    // Create a buffer to hold the ADC samples and clear it.
    // Note, we are going to sample two ADC inputs so they
    // end up in this buffer "interleaved". So you will want
    // a buffer twice this size to a real life given sample
    // frequency. See the printf() output for details.
    uint16_t adcInputBuffer[SAMPLE_BUFF_LEN];    
    memset(adcInputBuffer, 0, SAMPLE_BUFF_BYTES);
    
    // We use the ADC irq to trigger DMA and the manual says
    // that in this case the NVIC for ADC must be disabled.
    NVIC_DisableIRQ(ADC_IRQn);
    
    // Power up the ADC and set PCLK
    LPC_SC->PCONP    |=  (1UL << 12);
    LPC_SC->PCLKSEL0 &= ~(3UL << 24); // PCLK = CCLK/4 96M/4 = 24MHz
    
    // Enable the ADC, 12MHz,  ADC0.0 & .1
    //LPC_ADC->ADCR  = (1UL << 21) | (1UL << 8) | (3UL << 0); 
    LPC_ADC->ADCR  = (1UL << 21) | (1UL << 8) | (15UL << 0); 
    
    // Set the pin functions to ADC
    LPC_PINCON->PINSEL1 &= ~(3UL << 14);  /* P0.23, Mbed p15. */
    LPC_PINCON->PINSEL1 |=  (1UL << 14);
    LPC_PINCON->PINSEL1 &= ~(3UL << 16);  /* P0.24, Mbed p16. */
    LPC_PINCON->PINSEL1 |=  (1UL << 16);
    LPC_PINCON->PINSEL1 &= ~(3UL << 18);  /* P0.25, Mbed p17. */
    LPC_PINCON->PINSEL1 |=  (1UL << 18);
    LPC_PINCON->PINSEL1 &= ~(3UL << 20);  /* P0.26, Mbed p18. */
    LPC_PINCON->PINSEL1 |=  (1UL << 20);
  
    // Prepare an ADC configuration.
    MODDMA_Config *conf = new MODDMA_Config;
    conf
     ->channelNum    ( MODDMA::Channel_0 )
     ->srcMemAddr    ( 0 )
     ->dstMemAddr    ( (uint32_t)adcInputBuffer )  //NOT a dereference, just storing a 32bit pointer value as int
     ->transferSize  ( SAMPLE_BUFF_LEN )
     ->transferType  ( MODDMA::p2m )
     ->transferWidth ( MODDMA::word )
     ->srcConn       ( MODDMA::ADC )
     ->dstConn       ( 0 )
     ->dmaLLI        ( 0 )
     ->attach_tc     ( &TC0_callback )
     ->attach_err    ( &ERR0_callback )
    ; // end conf.
    
    // Prepare configuration.
    dma.Setup( conf );
    
    // Enable configuration.
    dma.Enable( conf );
    
    // Enable ADC irq flag (to DMA).
    // Note, don't set the individual flags,
    // just set the global flag.
    LPC_ADC->ADINTEN = 0x100;

    // Enable burst mode on inputs 0 and 1.
    LPC_ADC->ADCR |= (1UL << 16);

    
    while (1) {
        // When transfer complete do this block.
        if (dmaTransferComplete) {
            delete conf; // No memory leaks, delete the configuration.
            dmaTransferComplete = false;
        }
        //sock.sendTo(broadcast, out_buffer, sizeof(out_buffer));             
        sock.sendTo(broadcast, (char*)adcInputBuffer, SAMPLE_BUFF_BYTES);      
        wait(0.25);        
    }
}

// Configuration callback on TC
void TC0_callback(void) {
    
    MODDMA_Config *config = dma.getConfig();
    
    // Disbale burst mode and switch off the IRQ flag.
    LPC_ADC->ADCR &= ~(1UL << 16);
    LPC_ADC->ADINTEN = 0;    
    
    // Finish the DMA cycle by shutting down the channel.
    dma.haltAndWaitChannelComplete( (MODDMA::CHANNELS)config->channelNum());
    dma.Disable( (MODDMA::CHANNELS)config->channelNum() );
    
    // Tell main() while(1) loop to print the results.
    dmaTransferComplete = true;            
    
    // Switch on LED2 to show transfer complete.
    led2 = 1;        
    
    // Clear DMA IRQ flags.
    if (dma.irqType() == MODDMA::TcIrq) dma.clearTcIrq();    
    if (dma.irqType() == MODDMA::ErrIrq) dma.clearErrIrq();
}

// Configuration callback on Error
void ERR0_callback(void) {
    // Switch off burst conversions.
    LPC_ADC->ADCR |= ~(1UL << 16);
    LPC_ADC->ADINTEN = 0;
    error("Oh no! My Mbed EXPLODED! :( Only kidding, go find the problem");
}
