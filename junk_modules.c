/*
 * junk_modules.c
 *
 *  Created on: Mar 30, 2017
 *      Author: edsys
 */

			/*

			status = AD7124_NoCheckReadRegister(&myad7124, &ad7124_regs[AD7124_Data]);
			if(status){
			    transfer("D: ", debugConsole);
			    dec_ascii(num, ad7124_regs[AD7124_Data].value);
			    transfer(num, debugConsole);
			    transfer("\n\r", debugConsole);
			    }
			else{
				transfer("Error Reading data !! \n\r",debugConsole);
			}

		    AD7124_ADCControlConfig(&myad7124, &ad7124_regs);
			status = AD7124_NoCheckReadRegister(&myad7124, &ad7124_regs[AD7124_Data]);
			toggle = ~toggle;
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1, toggle);	// Toggle LED0 everytime a key is pressed
*/
