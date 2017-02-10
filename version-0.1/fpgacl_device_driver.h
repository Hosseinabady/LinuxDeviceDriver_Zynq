/* File: fpgacl_device_driver.h
 *
 Copyright (c) [2016] [Mohammad Hosseinabady (mohammad@hosseinabady.com)]
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===============================================================================
* This file has been written at University of Bristol
* for the ENPOWER project funded by EPSRC
*
* File name : fpgacl_device_driver.h
* author    : Mohammad hosseinabady mohammad@hosseinabady.com
* date      : 1 October 2016
* blog: https://highlevel-synthesis.com/
*/

#ifndef __FPGACL_DEVICE_DRIVER_H__
#define __FPGACL_DEVICE_DRIVER_H__


#define NPAGES 65536
#define MEM_DEVICE_NAME                                        "data_transfer_device"

#define DEVICE_NAME                                        "fpgacl_device"

#define FPGACL_DEVICE_NO_PORT                              1024


#define FPGACL_IOC_MAGIC                                   1000
#define FPGACL_DEVICE_IOWRITE                             _IOW(FPGACL_IOC_MAGIC, 0, int)
#define FPGACL_DEVICE_IOREAD                              _IOW(FPGACL_IOC_MAGIC, 1, int)
#define FPGACL_START                                      _IOW(FPGACL_IOC_MAGIC, 2, int)
#define FPGACL_CTRL_READY								  _IOW(FPGACL_IOC_MAGIC, 3, int)
#define FPGACL_CTRL_DONE                                  _IOW(FPGACL_IOC_MAGIC, 4, int)
#define FPGACL_INTERRUPT_ENABLE                           _IOW(FPGACL_IOC_MAGIC, 5, int)
#define FPGACL_REGISTER_DEVICE                            _IOW(FPGACL_IOC_MAGIC, 6, int)
#define FPGACL_MEMFREE                                    _IOW(FPGACL_IOC_MAGIC, 7, int)

struct argument_parameters_struct {
	unsigned int  size;
	unsigned int  type_size;
	unsigned int  register_kernel_base_address;
	unsigned int  register_physical_offset_address;
	unsigned int  index;
	long unsigned int irq_num;
	unsigned int  process_id;
};
typedef struct argument_parameters_struct argument_parameters_type;
//=========================================================================================
struct register_control_status_command_struct {
	void*             device_kernel_base_address;
	long unsigned int device_physical_base_address;
	long unsigned int register_physical_offset_address;
	long unsigned int irq_num;
	long unsigned int value;
	long unsigned int pointer;
	long unsigned int size_type;
	long unsigned int direction; //can be none = 0, in=1; out=2, inout=3
	unsigned int  process_id;
};
typedef struct register_control_status_command_struct register_control_status_command_type;
//=============================================================





#endif /* __FPGACL_DEVICE_DRIVER_H__ */
