#define SYSCTL_RCGUART 	(*((volatile unsigned long *)0x400FE618))
#define SYSCTL_PRUART 	(*((volatile unsigned long *)0x400FEA18))
#define SYSCTL_RCGCGPIO 	(*((volatile unsigned long *)0x400FE608))
#define SYSCTL_PRGPIO 	(*((volatile unsigned long *)0x400FEA08))

#define NVIC_ST_CTRL 	  (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD 	(*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT	(*((volatile unsigned long *)0xE000E018))
#define NVIC_SYS_PRI3	(*((volatile unsigned long *)0xE000ED20))
	
#define NVIC_EN0 	(*((volatile unsigned long *)0xE000E100))
#define NVIC_EN1 	(*((volatile unsigned long *)0xE000E104))
#define NVIC_PRI1 	(*((volatile unsigned long *)0xE000E404))	

#define GPIO_PORTF_LOCK 	(*((volatile unsigned long *)0x40025520))
#define UNLOCK 0x4C4F434B

#define GPIOA_DATA	(*((unsigned long *)0x400043FC))
#define GPIOA_DIR 	(*((volatile unsigned long *)0x40004400))
#define GPIOA_AFSEL 	(*((volatile unsigned long *)0x40004420))
#define GPIOA_PCTL 	(*((volatile unsigned long *)0x4000452C))
#define GPIOA_AMSEL 	(*((volatile unsigned long *)0x40004528))
#define GPIOA_PUR 	(*((volatile unsigned long *)0x40004510))
#define GPIOA_PDR 	(*((volatile unsigned long *)0x40004514))
#define GPIOA_DR8R 	(*((volatile unsigned long *)0x40004508))
#define GPIOA_DEN 	(*((volatile unsigned long *)0x4000451C))
#define GPIOA_ICR  	(*((unsigned long *)0x4000441C))

#define GPIOF_DATA	(*((unsigned long *)0x400253FC))
#define GPIOF_DIR 	(*((volatile unsigned long *)0x40025400))
#define GPIOF_AFSEL 	(*((volatile unsigned long *)0x40025420))
#define GPIOF_PCTL 	(*((volatile unsigned long *)0x4002552C))
#define GPIOF_AMSEL 	(*((volatile unsigned long *)0x40025528))
#define GPIOF_PUR 	(*((volatile unsigned long *)0x40025510))
#define GPIOF_PDR 	(*((volatile unsigned long *)0x40025514))
#define GPIOF_DR8R 	(*((volatile unsigned long *)0x40025508))
#define GPIOF_DEN 	(*((volatile unsigned long *)0x4002551C))
#define GPIOF_ICR  	(*((volatile unsigned long *)0x4002541C))
#define GPIOF_CR  	(*((volatile unsigned long *)0x40025524))
	

#define UART0_DR 	(*((volatile unsigned long *)0x4000C000))
#define UART0_FR 	(*((volatile unsigned long *)0x4000C018))
#define UART0_IBRD 	(*((volatile unsigned long *)0x4000C024))
#define UART0_FBRD 	(*((volatile unsigned long *)0x4000C028))
#define UART0_LCRH 	(*((volatile unsigned long *)0x4000C02C))
#define UART0_CTL 	(*((volatile unsigned long *)0x4000C030))

	
#define UART1_DR 	(*((volatile unsigned long *)0x4000D000))
#define UART1_FR 	(*((volatile unsigned long *)0x4000D018))
#define UART1_IBRD 	(*((volatile unsigned long *)0x4000D024))
#define UART1_FBRD 	(*((volatile unsigned long *)0x4000D028))
#define UART1_LCRH 	(*((volatile unsigned long *)0x4000D02C))
#define UART1_CTL 	(*((volatile unsigned long *)0x4000D030))
#define UART1_IM 	(*((volatile unsigned long *)0x4000D038))
#define UART1_ICR 	(*((volatile unsigned long *)0x4000D044))
#define UART1_FLS 	(*((volatile unsigned long *)0x4000D034))

#define GPIOE_DIR (*((volatile unsigned long *)0x40024400))
#define GPIOE_AFSEL 	(*((volatile unsigned long *)0x40024420))
#define GPIOE_PCTL 	(*((volatile unsigned long *)0x4002452C))
#define GPIOE_DEN 	(*((volatile unsigned long *)0x4002451C))
	
#define SYSCTL_RCGCTIMER 	(*((volatile unsigned long *)0X400FE604))
#define SYSCTL_PRTIMER 	(*((volatile unsigned long *)0X400FEA04))
#define TIMER0_CTL 	(*((volatile unsigned long *)0X4003000C	))
#define TIMER1_CTL 	(*((volatile unsigned long *)0X4003100C))
#define TIMER2_CTL 	(*((volatile unsigned long *)0X4003200C))	
#define TIMER0_CFG 	(*((volatile unsigned long *)0X40030000))
#define TIMER1_CFG 	(*((volatile unsigned long *)0X40031000))	
#define TIMER2_CFG 	(*((volatile unsigned long *)0X40032000))		
#define TIMER0_TAMR (*((volatile unsigned long *)0X40030004))	
#define TIMER0_TBMR (*((volatile unsigned long *)0X40030008))	
#define TIMER1_TAMR (*((volatile unsigned long *)0X40031004))	
#define TIMER1_TBMR (*((volatile unsigned long *)0X40031008))
#define TIMER2_TAMR (*((volatile unsigned long *)0X40032004))	
#define TIMER0_TAILR (*((volatile unsigned long *)0X40030028))
#define TIMER0_TBILR (*((volatile unsigned long *)0X4003002C))
#define TIMER1_TAILR (*((volatile unsigned long *)0X40031028))
#define TIMER1_TBILR (*((volatile unsigned long *)0X4003102C))	
#define TIMER2_TAILR (*((volatile unsigned long *)0X40032028))
#define TIMER0_TAPMR (*((volatile unsigned long *)0X40030040))	
#define TIMER0_TBPMR (*((volatile unsigned long *)0X40030044))
#define TIMER2_TAPMR (*((volatile unsigned long *)0X40032040))	
#define TIMER1_TAPMR (*((volatile unsigned long *)0X40031040))
#define TIMER1_TBPMR (*((volatile unsigned long *)0X40031044))
#define TIMER0_TAMATCH (*((volatile unsigned long *)0X40030030))
#define TIMER0_TBMATCH (*((volatile unsigned long *)0X40030034))
#define TIMER1_TAMATCH (*((volatile unsigned long *)0X40031030))	
#define TIMER1_TBMATCH (*((volatile unsigned long *)0X40031034))	
#define TIMER2_TAV (*((volatile unsigned long *)0X40032050))	



#define RCGI2C (*((volatile unsigned long *)0X400FE620))
#define GPIOB_AFSEL (*((volatile unsigned long *)0X40005420))
#define GPIOB_DEN (*((volatile unsigned long *)0X4000551C))
#define GPIOB_DIR (*((volatile unsigned long *)0X40005400))	
#define GPIOB_ODR (*((volatile unsigned long *)0X4000550C))
#define GPIOB_PCTL (*((volatile unsigned long *)0X4000552C))
#define GPIOB_LOCK (*((volatile unsigned long *)0X40005520))

#define I2CMCR (*((volatile unsigned long *)0X40020020))
#define I2CMTPR (*((volatile unsigned long *)0X4002000C))
#define I2CMSA (*((volatile unsigned long *)0X40020000))
#define I2CMDR (*((volatile unsigned long *)0X40020008))	
#define I2CMCS (*((volatile unsigned long *)0X40020004))
#define I2CCLKOCNT (*((volatile unsigned long *)0X40020024))
#define SRI2C *((volatile unsigned long *)0X400FE520))
#define PRI2C *((volatile unsigned long *)0X400FEA20))

	