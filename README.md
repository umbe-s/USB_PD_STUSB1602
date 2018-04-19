# USB_PD_STUSB1602
Code for the STM32F405 micro to drive USB PD negotiations via the STUSB1602.
This code was first developed by Design Concepts, Inc. in Madison, WI. 

ST Micro provides static libraries with which to do USB-PD negotiation with the STUSB1602, but these can only be used with the STM32F072. For any other processor, there is precious little help. 

In its current form, this code (with minor tweaks for the use-case) can be used to do basic USB PD negotiations with an STM32F405 and an STUSB1602. It is not a standalone solution, and does not come close to meeting the USB PD specification. With some help we might expand it to support more STM32 families and/or meet more of the USB PD specification.
