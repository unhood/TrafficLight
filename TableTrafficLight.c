// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"
#include "assert.h"

#define USE_SOLUTION_BIG (1)

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void Delay100ms(unsigned char);
#define NORTH_GREEN_EAST_RED  (0x21)
#define NORTH_YELLOW_EAST_RED (0x22)
#define NORTH_RED_EAST_GREEN  (0x0C)
#define NORTH_RED_EAST_YELLOW (0x14)
#define NORTH_RED_EAST_RED    (0x24)
#define WALK_ON               (0x08)
#define DWALK_ON_WALK_OFF     (0x02)
#define DWALK_OFF_WALK_OFF    (0x00)
#define SENSORS_MASK          (0x7)
#define GO_DELAY_FACTOR       (20) //50 - 5 sec
#define WAIT_DELAY_FACTOR     (10) //10 - 1 sec
#define FLASH_DELAY_DIV       (5)  //must be <= (WAIT_DELAY_FACTOR/2)
#define FLASH_DELAY_FACTOR    (WAIT_DELAY_FACTOR/(FLASH_DELAY_DIV*2))

#if (USE_SOLUTION_BIG)
// State Event Matrix framework
#define EVENT_NULL (0xF)
typedef unsigned char sem_event;
typedef unsigned char sem_state;
typedef sem_event (*sem_handler)(sem_event);

typedef struct sem_table
{
	sem_event     Event;
	unsigned char HandlerIdx;
	sem_state     NextState;
} sem_table;

#define M_CvtNameToIdx(Handler)              INDEX_##Handler
#define M_PackSEM(Event, Handler, NextState) {(sem_event)(Event),(unsigned char)(INDEX_##Handler),(sem_state)(NextState)}
#define M_GetEvent(TableEntry)               ((TableEntry)->Event)
#define M_GetNextState(TableEntry)           ((TableEntry)->NextState)
#define M_GetHandler(Entry,FuncTable,Event)  ((FuncTable)[(Entry)->HandlerIdx]((sem_event)(Event)))

__inline const sem_table *MatchEvent(const sem_table * const SEMTable[], sem_state State, sem_event Event)
{
	const sem_table *SEMTableEntry = SEMTable[State];
	int i=0;
	for (i=0; 
		  ((sem_event)(M_GetEvent(&(SEMTableEntry[i]))) != Event) && ((sem_event)(M_GetEvent(&(SEMTableEntry[i]))) != EVENT_NULL); 
	     i++)
	{
		// do nothing
	}
	return (&SEMTableEntry[i]);
}
// Handler function prototype
static sem_event GoNorthLight(sem_event);
static sem_event WaitNorthLight(sem_event);
static sem_event GoEastLight(sem_event);
static sem_event WaitEastLight(sem_event);
static sem_event GoWalkLight(sem_event);
static sem_event WaitWalkLight(sem_event);
static sem_event NullHandler(sem_event);

// This Traffic statemachine
typedef enum traffic_state
{
	STATE_GO_NORTH,
	STATE_WAIT_NORTH,
	STATE_GO_EAST,
	STATE_WAIT_EAST,
	STATE_GO_WALK,
	STATE_WAIT_WALK,
	STATE_MAX
} traffic_state;

typedef enum traffic_event
{                        // PE210
	EVENT_EMPTY_ROAD,      // 000 - No sensor triggered
	EVENT_EAST,            // 001 - East/West car sensor triggered
	EVENT_NORTH,           // 010 - North/South car sensor triggered
	EVENT_NORTH_EAST,      // 011 - both North/South and East/West car sensors triggered
	EVENT_WALK,            // 100 - Pedestrian sensor triggered
	EVENT_WALK_EAST,       // 101 - both Pedestrian and East/West car sensors triggered
	EVENT_WALK_NORTH,      // 110 - both Pedestrian and North/South car sensors triggered
	EVENT_WALK_NORTH_EAST  // 111 - all sensors triggered
} traffic_event;

typedef enum traffic_handler_idx
{
	M_CvtNameToIdx( GoNorthLight ),
	M_CvtNameToIdx( WaitNorthLight ),
	M_CvtNameToIdx( GoEastLight ),
	M_CvtNameToIdx( WaitEastLight ),
	M_CvtNameToIdx( GoWalkLight ),
	M_CvtNameToIdx( WaitWalkLight ),
	M_CvtNameToIdx( NullHandler )
} traffic_handler_idx;

// State Machine Table
const sem_handler TrafficHandlers[] =
{
	// These must be in the same order as they are listed in traffic_handler_idx
	GoNorthLight,
	WaitNorthLight,
	GoEastLight,
	WaitEastLight,
	GoWalkLight,
	WaitWalkLight,
	NullHandler
};
const sem_table StateGoNorth[]=
{
	M_PackSEM( EVENT_EMPTY_ROAD,      GoNorthLight,      STATE_GO_NORTH   ),
	M_PackSEM( EVENT_EAST,            WaitNorthLight,    STATE_WAIT_NORTH ),
	M_PackSEM( EVENT_NORTH,           GoNorthLight,      STATE_GO_NORTH   ),
	M_PackSEM( EVENT_NORTH_EAST,      WaitNorthLight,    STATE_WAIT_NORTH ),
	M_PackSEM( EVENT_WALK,            WaitNorthLight,    STATE_WAIT_NORTH ),
	M_PackSEM( EVENT_WALK_EAST,       WaitNorthLight,    STATE_WAIT_NORTH ),
	M_PackSEM( EVENT_WALK_NORTH,      WaitNorthLight,    STATE_WAIT_NORTH ),
	M_PackSEM( EVENT_WALK_NORTH_EAST, WaitNorthLight,    STATE_WAIT_NORTH ),
	M_PackSEM( EVENT_NULL,            NullHandler,       STATE_GO_NORTH   )
};
const sem_table StateWaitNorth[]=
{
	M_PackSEM( EVENT_EMPTY_ROAD,      GoEastLight,       STATE_GO_EAST    ), // just go East anyway
	M_PackSEM( EVENT_EAST,            GoEastLight,       STATE_GO_EAST    ),
	M_PackSEM( EVENT_NORTH,           GoEastLight,       STATE_GO_EAST    ), // just go East anyway
	M_PackSEM( EVENT_NORTH_EAST,      GoEastLight,       STATE_GO_EAST    ),
	M_PackSEM( EVENT_WALK,            GoWalkLight,       STATE_GO_WALK    ),
	M_PackSEM( EVENT_WALK_EAST,       GoWalkLight,       STATE_GO_WALK    ),
	M_PackSEM( EVENT_WALK_NORTH,      GoWalkLight,       STATE_GO_WALK    ),
	M_PackSEM( EVENT_WALK_NORTH_EAST, GoWalkLight,       STATE_GO_WALK    ),
	M_PackSEM( EVENT_NULL,            NullHandler,       STATE_WAIT_NORTH )
};
const sem_table StateGoEast[]=
{
	M_PackSEM( EVENT_EMPTY_ROAD,      GoEastLight,       STATE_GO_EAST    ),
	M_PackSEM( EVENT_EAST,            GoEastLight,       STATE_GO_EAST    ),
	M_PackSEM( EVENT_NORTH,           WaitEastLight,     STATE_WAIT_EAST  ),
	M_PackSEM( EVENT_NORTH_EAST,      WaitEastLight,     STATE_WAIT_EAST  ),
	M_PackSEM( EVENT_WALK,            WaitEastLight,     STATE_WAIT_EAST  ),
	M_PackSEM( EVENT_WALK_EAST,       WaitEastLight,     STATE_WAIT_EAST  ),
	M_PackSEM( EVENT_WALK_NORTH,      WaitEastLight,     STATE_WAIT_EAST  ),
	M_PackSEM( EVENT_WALK_NORTH_EAST, WaitEastLight,     STATE_WAIT_EAST  ),
	M_PackSEM( EVENT_NULL,            NullHandler,       STATE_GO_EAST    )
};
const sem_table StateWaitEast[]=
{
	M_PackSEM( EVENT_EMPTY_ROAD,      GoNorthLight,      STATE_GO_NORTH   ), // just go North anyway
	M_PackSEM( EVENT_EAST,            GoNorthLight,      STATE_GO_NORTH   ), // just go North anyway
	M_PackSEM( EVENT_NORTH,           GoNorthLight,      STATE_GO_NORTH   ),
	M_PackSEM( EVENT_NORTH_EAST,      GoNorthLight,      STATE_GO_NORTH   ),
	M_PackSEM( EVENT_WALK,            GoWalkLight,       STATE_GO_WALK    ),
	M_PackSEM( EVENT_WALK_EAST,       GoWalkLight,       STATE_GO_WALK    ),
	M_PackSEM( EVENT_WALK_NORTH,      GoNorthLight,      STATE_GO_NORTH   ),
	M_PackSEM( EVENT_WALK_NORTH_EAST, GoNorthLight,      STATE_GO_NORTH   ),
	M_PackSEM( EVENT_NULL,            NullHandler,       STATE_WAIT_EAST  )
};
const sem_table StateGoWalk[]=
{
	M_PackSEM( EVENT_EMPTY_ROAD,      GoWalkLight,       STATE_GO_WALK    ),
	M_PackSEM( EVENT_EAST,            WaitWalkLight,     STATE_WAIT_WALK  ),
	M_PackSEM( EVENT_NORTH,           WaitWalkLight,     STATE_WAIT_WALK  ),
	M_PackSEM( EVENT_NORTH_EAST,      WaitWalkLight,     STATE_WAIT_WALK  ),
	M_PackSEM( EVENT_WALK,            GoWalkLight,       STATE_GO_WALK    ),
	M_PackSEM( EVENT_WALK_EAST,       WaitWalkLight,     STATE_WAIT_WALK  ),
	M_PackSEM( EVENT_WALK_NORTH,      WaitWalkLight,     STATE_WAIT_WALK  ),
	M_PackSEM( EVENT_WALK_NORTH_EAST, WaitWalkLight,     STATE_WAIT_WALK  ),
	M_PackSEM( EVENT_NULL,            NullHandler,       STATE_GO_WALK    )
};
const sem_table StateWaitWalk[]=
{
	M_PackSEM( EVENT_EMPTY_ROAD,      GoEastLight,       STATE_GO_EAST    ), // just go East (as round sequence) anyway
	M_PackSEM( EVENT_EAST,            GoEastLight,       STATE_GO_EAST    ),
	M_PackSEM( EVENT_NORTH,           GoNorthLight,      STATE_GO_NORTH   ),
	M_PackSEM( EVENT_NORTH_EAST,      GoEastLight,       STATE_GO_EAST    ),
	M_PackSEM( EVENT_WALK,            GoEastLight,       STATE_GO_EAST    ), // just go East (as round sequence) anyway
	M_PackSEM( EVENT_WALK_EAST,       GoEastLight,       STATE_GO_EAST    ),
	M_PackSEM( EVENT_WALK_NORTH,      GoNorthLight,      STATE_GO_NORTH   ),
	M_PackSEM( EVENT_WALK_NORTH_EAST, GoEastLight,       STATE_GO_EAST    ),
	M_PackSEM( EVENT_NULL,            NullHandler,       STATE_WAIT_WALK  )
};

const sem_table *const TrafficSEM[STATE_MAX]=
{
	// These must be in the same order as they are listed in traffic_state
	StateGoNorth,
	StateWaitNorth,
	StateGoEast,
	StateWaitEast,
	StateGoWalk,
	StateWaitWalk
};

// Struct to track the current state and event of the statemachine
typedef struct traffic_operation
{
	traffic_state State;
	traffic_event Event;
} traffic_operation;

// Global variable to track the current state and event
static traffic_operation TrafficOp;

// ***** 3. Subroutines Section *****
static sem_event GoNorthLight(sem_event Event)
{
	sem_event NextEvent;
	// Set traffic lights
	GPIO_PORTB_DATA_R = NORTH_GREEN_EAST_RED;
	GPIO_PORTF_DATA_R = DWALK_ON_WALK_OFF;
	// 5 seconds
	Delay100ms(GO_DELAY_FACTOR);
	// check sensor
	NextEvent = (GPIO_PORTE_DATA_R & SENSORS_MASK);
	return (NextEvent);
}
static sem_event WaitNorthLight(sem_event Event)
{
	sem_event NextEvent;
	// Set traffic lights
	GPIO_PORTB_DATA_R = NORTH_YELLOW_EAST_RED;
	GPIO_PORTF_DATA_R = DWALK_ON_WALK_OFF;
	// 1 seconds
  Delay100ms(WAIT_DELAY_FACTOR);
	// check sensor
	NextEvent = (GPIO_PORTE_DATA_R & SENSORS_MASK);
  return (NextEvent);	
}
static sem_event GoEastLight(sem_event Event)
{
	sem_event NextEvent;
	// Set traffic lights
	GPIO_PORTB_DATA_R = NORTH_RED_EAST_GREEN;
	GPIO_PORTF_DATA_R = DWALK_ON_WALK_OFF;
	// 5 seconds
	Delay100ms(GO_DELAY_FACTOR);
	// check sensor
	NextEvent = (GPIO_PORTE_DATA_R & SENSORS_MASK);
	return (NextEvent);
}
static sem_event WaitEastLight(sem_event Event)
{
	sem_event NextEvent;
	// Set traffic lights
	GPIO_PORTB_DATA_R = NORTH_RED_EAST_YELLOW;
	GPIO_PORTF_DATA_R = DWALK_ON_WALK_OFF;
	// 1 seconds
	Delay100ms(WAIT_DELAY_FACTOR);
	// check sensor
	NextEvent = (GPIO_PORTE_DATA_R & SENSORS_MASK);
	return (NextEvent);
}
static sem_event GoWalkLight(sem_event Event)
{
	sem_event NextEvent;
	// Set traffic lights
	GPIO_PORTB_DATA_R = NORTH_RED_EAST_RED;
	GPIO_PORTF_DATA_R = WALK_ON;
	// 5 seconds
	Delay100ms(GO_DELAY_FACTOR);
	NextEvent = (GPIO_PORTE_DATA_R & SENSORS_MASK);
	return (NextEvent);
}
static sem_event WaitWalkLight(sem_event Event)
{
	sem_event NextEvent;
	unsigned int i = FLASH_DELAY_DIV;
	// Set traffic lights
	GPIO_PORTB_DATA_R = NORTH_RED_EAST_RED;
	// Flashing the Dont walk light
	do
	{
	  GPIO_PORTF_DATA_R = DWALK_ON_WALK_OFF;
	  Delay100ms(FLASH_DELAY_FACTOR);
	  GPIO_PORTF_DATA_R = DWALK_OFF_WALK_OFF;
	  Delay100ms(FLASH_DELAY_FACTOR);
		i--;
	} while(i>0);
	// check sensor
	NextEvent = (GPIO_PORTE_DATA_R & SENSORS_MASK);
	return (NextEvent);
}
static sem_event NullHandler(sem_event Event)
{
	// we should not come here
	// always assert
	assert(0);
	return ( Event );
}
#else // #if (USE_SOLUTION_BIG)
typedef enum walk_led_mode
{
  WALK_LED_MODE_STOP,   //(0)
  WALK_LED_MODE_WALK,   //(1)
  WALK_LED_MODE_FLASH  //(2)
} walk_led_mode;

__inline void WalkLedControl(walk_led_mode Mode);

typedef struct state
{
	unsigned char       CarLed;
	walk_led_mode       WalkLedMode;
	unsigned long       Time;
	const struct state *NextState[8];
} state;

#define GO_NORTH    &StateMachine[0]
#define WAIT_NORTH  &StateMachine[1]
#define GO_EAST     &StateMachine[2]
#define WAIT_EAST   &StateMachine[3]
#define GO_WALK     &StateMachine[4]
#define WAIT_WALK   &StateMachine[5]

const state StateMachine[6]=
{
	// GO_NORTH
  {
		NORTH_GREEN_EAST_RED,
		WALK_LED_MODE_STOP,
		GO_DELAY_FACTOR,
		{
			GO_NORTH,  //000
			WAIT_NORTH,//001
			GO_NORTH,  //010
			WAIT_NORTH,//011
			WAIT_NORTH,//100
			WAIT_NORTH,//101
			WAIT_NORTH,//110
			WAIT_NORTH //111
		}
	}, 
	// WAIT_NORTH
	{
		NORTH_YELLOW_EAST_RED,
		WALK_LED_MODE_STOP,
		WAIT_DELAY_FACTOR,
		{
			GO_EAST,//000
			GO_EAST,//001
			GO_EAST,//010
			GO_EAST,//011
			GO_WALK,//100
			GO_EAST,//101
			GO_WALK,//110
			GO_WALK //111
		}
	}, 
	// GO_EAST
	{
		NORTH_RED_EAST_GREEN,
		WALK_LED_MODE_STOP,
		GO_DELAY_FACTOR,
		{
			GO_EAST,  //000
			GO_EAST,  //001
			WAIT_EAST,//010
			WAIT_EAST,//011
			WAIT_EAST,//100
			WAIT_EAST,//101
			WAIT_EAST,//110
			WAIT_EAST //111
		}
	}, 
	// WAIT_EAST
	{
		NORTH_RED_EAST_YELLOW,
		WALK_LED_MODE_STOP,
		WAIT_DELAY_FACTOR,
		{
			GO_NORTH,//000
			GO_NORTH,//001
			GO_NORTH,//010
			GO_NORTH,//011
			GO_WALK, //100
			GO_WALK, //101
			GO_NORTH,//110
			GO_NORTH //111
		}
	}, 
	// GO_WALK
	{
		NORTH_RED_EAST_RED,
		WALK_LED_MODE_WALK,
		GO_DELAY_FACTOR,
		{
			GO_WALK,  //000
			WAIT_WALK,//001
			WAIT_WALK,//010
			WAIT_WALK,//011
			GO_WALK,  //100
			WAIT_WALK,//101
			WAIT_WALK,//110
			WAIT_WALK //111
		}
	}, 
	// WAIT_WALK
	{
		NORTH_RED_EAST_RED,
		WALK_LED_MODE_FLASH,
		FLASH_DELAY_FACTOR,
		{
			GO_EAST, //000
			GO_EAST, //001
			GO_NORTH,//010
			GO_EAST, //011
			GO_EAST, //100
			GO_EAST, //101
			GO_NORTH,//110
			GO_EAST  //111
		}
	}, 
};
__inline void WalkLedControl( walk_led_mode Mode )
{
	switch ( Mode )
	{
		case WALK_LED_MODE_STOP:
		{
			GPIO_PORTF_DATA_R = DWALK_ON_WALK_OFF;
		  break;
		}
		case WALK_LED_MODE_WALK:
		{
		  GPIO_PORTF_DATA_R = WALK_ON;
		  break;
		}
		case WALK_LED_MODE_FLASH:
		{
			unsigned int i = FLASH_DELAY_DIV - 1; 
			do
			{
		    GPIO_PORTF_DATA_R = DWALK_ON_WALK_OFF;
			  Delay100ms(FLASH_DELAY_FACTOR);
			  GPIO_PORTF_DATA_R = DWALK_OFF_WALK_OFF;
				Delay100ms(FLASH_DELAY_FACTOR);
				i--;
			} while(i>0);
			GPIO_PORTF_DATA_R = DWALK_ON_WALK_OFF;
		  break;
		}
		default:
			break;
	}
}
const state *TrafficOp;

#endif // #if (USE_SOLUTION_BIG) #else
void Delay100ms(unsigned char time)
{
	while (time>0)
	{
		NVIC_ST_RELOAD_R = 0x7A11FF; // 8000000 - 1= 7999999
		NVIC_ST_CURRENT_R = 0x0;     // write to reset clock
		while ((NVIC_ST_CTRL_R & 0x10000) == 0)
		{
			// do nothing
		}
		time--;
	}
}
void InitSysTick(void)
{
	NVIC_ST_CTRL_R = 0x0; //disable during setup
	NVIC_ST_CTRL_R = 0x5; //enable systick with core clock
}
void InitPortBEF(void)
{
	unsigned long delay;
	SYSCTL_RCGC2_R |= 0x32;
	delay = SYSCTL_RCGC2_R;
	// Set port E - sensors
	GPIO_PORTE_DIR_R   &= ~0x7;  // set PE0,1,2 as input
	GPIO_PORTE_AFSEL_R &= ~0x7;// not alternate
	GPIO_PORTE_AMSEL_R &= ~0x7;// not analog
	GPIO_PORTE_PCTL_R  &= ~(0xFFF);//bits for PE0,1,2
	GPIO_PORTE_DEN_R   |= 0x7; //enable pins 0,1,2 as digital pins
	// Set port F - walk leds
	GPIO_PORTF_LOCK_R = 0x4C4F434B; //unlock gpio port f
	GPIO_PORTF_CR_R |= 0xA; //allow changes to PF1,PF3
	GPIO_PORTF_DIR_R   |= 0xA;  // set PF1,2 as output
	GPIO_PORTF_AFSEL_R &= ~0xA;// not alternate
	GPIO_PORTF_AMSEL_R &= ~0xA;// not analog
	GPIO_PORTF_PCTL_R  &= ~(0xF0F0);//bits for PF1,1
	GPIO_PORTF_DEN_R   |= 0xA; //enable pins 1,3 as digital pins
	// Set Port B - car leds
	GPIO_PORTB_LOCK_R = 0x4C4F434B; //unlock gpio port f
	GPIO_PORTB_CR_R |= 0x3F; //allow changes to PB012345
	GPIO_PORTB_DIR_R   |= 0x3F;  // set PB012345 as output
	GPIO_PORTB_AFSEL_R &= ~0x3F;// not alternate
	GPIO_PORTB_AMSEL_R &= ~0x3F;// not analog
	GPIO_PORTB_PCTL_R  &= ~(0xFFFFFF);//bits for PB012345
	GPIO_PORTB_DEN_R   |= 0x3F; //enable pins 0,1,2 as digital pins
	
}

int main(void){ 
#if (USE_SOLUTION_BIG)
	const sem_table *TableEntry;
#else
	TrafficOp = &StateMachine[0];
#endif	
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
  // Init SysTick
	InitSysTick();
	// Init port B E F
  InitPortBEF();
	
  EnableInterrupts();

  while(1){
  #if (USE_SOLUTION_BIG)
    TableEntry = MatchEvent(TrafficSEM, TrafficOp.State,(sem_event)TrafficOp.Event);
		TrafficOp.Event = (traffic_event) M_GetHandler(TableEntry, TrafficHandlers, TrafficOp.Event);
		TrafficOp.State = (traffic_state) M_GetNextState(TableEntry);
	#else
		// Car led
		GPIO_PORTB_DATA_R = TrafficOp->CarLed;
		// Walk led
		WalkLedControl(TrafficOp->WalkLedMode);
		// Wait
		Delay100ms(TrafficOp->Time);
		// Next state
		TrafficOp = TrafficOp->NextState[GPIO_PORTE_DATA_R & SENSORS_MASK];
  #endif // #if (USE_SOLUTION_BIG) #else
  }
}

