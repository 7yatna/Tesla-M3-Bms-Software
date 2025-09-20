/*
 * This file is part of the stm32-template project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/desig.h>
#include "stm32_can.h"
#include "canmap.h"
#include "cansdo.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
#include "anain.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "terminalcommands.h"
#include "my_string.h"
#include "BatMan.h"
#include "ModelS.h"
#include "BMSUtil.h"
#include "MAXbms.h"
#include "isa_shunt.h"
#define PRINT_JSON 0

			   
extern "C" void __cxa_pure_virtual()
{
    while (1);
}

static Stm32Scheduler* scheduler;
static CanHardware* can;
static CanMap* canMap;
static CanSdo* canSdo;
static uint8_t BMStype;
int uauxGain = 222;	
uint8_t Gcount = 0x00;
float SOCVal = 0;

//sample 10 ms task
static void Ms10Task(void)
{
    //Set timestamp of error message
    ErrorMessage::SetTime(rtc_get_counter_val());
	ProcessUdc();
}

void ProcessUdc()
{
    if (Param::GetInt(Param::ShuntType) == 1)//ISA shunt
    {
        float udc1 = ((float)ISA::Voltage)/1000;//get voltage from isa sensor and post to parameter database
        Param::SetFloat(Param::udc, udc1);
        float udc2 = ((float)ISA::Voltage2)/1000;//get voltage from isa sensor and post to parameter database
        Param::SetFloat(Param::udc2, udc2);
        float udc3 = ((float)ISA::Voltage3)/1000;//get voltage from isa sensor and post to parameter database
        Param::SetFloat(Param::udc3, udc3);
        float idc = ((float)ISA::Amperes)/1000;//get current from isa sensor and post to parameter database
        Param::SetFloat(Param::idc, idc);
        float kw = ((float)ISA::KW)/1000;//get power from isa sensor and post to parameter database
        Param::SetFloat(Param::power, kw);
        float kwh = ((float)ISA::KWh)/1000;//get kwh from isa sensor and post to parameter database
        Param::SetFloat(Param::KWh, kwh);
        float Amph = ((float)ISA::Ah)/3600;//get Ah from isa sensor and post to parameter database
        Param::SetFloat(Param::AMPh, Amph);
	}
}

void CalcSOC()
{
    float Capacity_Parm = Param::GetFloat(Param::BattCap);
    float kwh_Used = ABS(Param::GetFloat(Param::KWh));

    SOCVal = 100.0f - 100.0f * kwh_Used / Capacity_Parm;

    if(SOCVal > 100) SOCVal = 100;
    Param::SetFloat(Param::SOC,SOCVal);
}
	
//sample 100ms task
static void Ms100Task(void)
{
    DigIo::led_out.Toggle();
    iwdg_reset();
	Param::SetInt(Param::IN1, DigIo::in1.Get());
	Param::SetInt(Param::IN2, DigIo::in2.Get());
	if(DigIo::in1.Get())DigIo::out1.Set(); else DigIo::out1.Clear();
	if(DigIo::in2.Get())DigIo::out2.Set(); else DigIo::out2.Clear();
	Param::SetInt(Param::CoolantPUMP, DigIo::out1.Get());
	Param::SetInt(Param::CoolantFAN, DigIo::out2.Get());
	Param::SetFloat(Param::uaux, ((float)AnaIn::lvmon.Get()) / uauxGain);
    float cpuLoad = scheduler->GetCpuLoad();
    Param::SetFloat(Param::cpuload, cpuLoad / 10);
	int32_t IsaTemp=ISA::Temperature;
    Param::SetInt(Param::tmpaux,IsaTemp);
	/*
	if(Param::GetInt(Param::ShuntType) != 0)//Do not do any SOC calcs
    {
        CalcSOC();
    }
	*/
//!!! to change to BMS class with selectable types under it to clean up code and simplify interactions//
    if(BMStype == BMS_M3)
    {
        BATMan::loop();
    }
    else if(BMStype == BMS_MAX)
    {
        MAXbms::Task100Ms();
    }
///////////////
    BMSUtil::UpdateSOC();
    canMap->SendAll();
	Can_Tasks();
	timer_set_period(TIM3, Param::GetInt(Param::Tim_Period));
	timer_set_prescaler(TIM3,Param::GetInt(Param::Tim_Presc));
    timer_set_oc_value(TIM3, TIM_OC3, Param::GetInt(Param::Tim_1_OC));
    timer_set_oc_value(TIM3, TIM_OC4, Param::GetInt(Param::Tim_2_OC));
}

static void Ms200Task(void)
{
	
}

void Can_Tasks()
{
	
	uint8_t bytes[8];
    bytes[0]= 0x00;
    bytes[1]= 0x00;
	bytes[2]= Gcount;
    bytes[3]= 0x0A;
    
    can->Send(0x1D2, bytes, 4); //Send on CAN1	
	
	Gcount = Gcount + 0x01;
   
   if (Gcount==0xFF)
   {
      Gcount=0x00;
   }
}

void DecodeCAN(int id, uint32_t* data)
{
	uint8_t* bytes = (uint8_t*)data;
	switch (id)
	{
		case 0x1AE:
			Param::SetInt(Param::opmode, bytes[0]);
			break;
		default:
			break;
	}
			
}
static void SetCanFilters()
{
	can->RegisterUserMessage(0x605); //Can SDO
	can->RegisterUserMessage(0x1AE); //OI Control Message
	if (Param::GetInt(Param::ShuntType) == 1)  ISA::RegisterCanMessages(0);
}
	
static bool CanCallback(uint32_t id, uint32_t data[2], uint8_t dlc) //This is where we go when a defined CAN message is received.
{
    dlc = dlc;
	if (Param::GetInt(Param::CanCtrl)) DecodeCAN(id,data);
	if (Param::GetInt(Param::ShuntType) == 1)  ISA::DecodeCAN(id, data);	
	return false;
}

/** This function is called when the user changes a parameter */


void Param::Change(Param::PARAM_NUM paramNum)
{
    switch (paramNum)
    {
		case Param::NodeId:
			canSdo->SetNodeId(Param::GetInt(Param::NodeId));
			break; 
		case Param::Tim_Presc:
		case Param::Tim_Period:
		case Param::Tim_1_OC:
		case Param::Tim_2_OC:
			tim3_setup();
			break;
		case Param::CanCtrl:
		case Param::ShuntType:
			SetCanFilters();
			break;
    default:
        //Handle general parameter changes here. Add paramNum labels for handling specific parameters
        break;
    }
}

//Whichever timer(s) you use for the scheduler, you have to
//implement their ISRs here and call into the respective scheduler
extern "C" void tim2_isr(void)
{
    scheduler->Run();
}

extern "C" int main(void)
{
    extern const TERM_CMD termCmds[];

    clock_setup(); //Must always come first
    rtc_setup();
    ANA_IN_CONFIGURE(ANA_IN_LIST);
    DIG_IO_CONFIGURE(DIG_IO_LIST);
    AnaIn::Start(); //Starts background ADC conversion via DMA
    write_bootloader_pininit(); //Instructs boot loader to initialize certain pins
	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_CAN1_REMAP_PORTB);//Remap CAN pins to Portb alt funcs.
    nvic_setup(); //Set up some interrupts
    parm_load(); //Load stored parameters
    spi1_setup();// SPI1 for Model 3 BMB modules
	tim3_setup();
    usart1_setup();//Usart 1 for Model S / X slaves
    Stm32Scheduler s(TIM2); //We never exit main so it's ok to put it on stack
    scheduler = &s;

    //Initialize CAN1, including interrupts. Clock must be enabled in clock_setup()
    Stm32Can c(CAN1, CanHardware::Baud500,true);
    FunctionPointerCallback cb(CanCallback, SetCanFilters);
	CanMap cm(&c);
	CanSdo sdo(&c, &cm);
	sdo.SetNodeId(Param::GetInt(Param::NodeId));
//store a pointer for easier access
	can = &c;
    canMap = &cm;
    canSdo = &sdo;
	c.AddCallback(&cb);
    Terminal t(USART3, termCmds);
    TerminalCommands::SetCanMap(canMap);
    BATMan::BatStart();
    s.AddTask(Ms10Task, 10);
    s.AddTask(Ms100Task, 100);
	s.AddTask(Ms200Task, 200);
	
	if(Param::GetInt(Param::IsaInit)==1) ISA::initialize(0);//only call this once if a new sensor is fitted.


	if(BMStype == BMS_M3)
    {
        BATMan::BatStart();
    }
    else if(BMStype == BMS_MAX)
    {
        MAXbms::MaxStart();
    } 

    Param::SetInt(Param::version, 4);
    Param::Change(Param::PARAM_LAST); //Call callback one for general parameter propagation
	SetCanFilters();
	if (Param::GetInt(Param::CanCtrl) == 0)
	{
		Param::SetInt(Param::opmode, 0);
	}
    while(1)
    {
        char c = 0;
        t.Run();
        if (sdo.GetPrintRequest() == PRINT_JSON)
        {
            TerminalCommands::PrintParamsJson(&sdo, &c);
        }
    }

    return 0;
}

