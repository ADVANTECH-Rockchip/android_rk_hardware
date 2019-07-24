/*******************************************************************************
 *
 *  Copyright (C) 2009-2011 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/*****************************************************************************
**                                                                           
**  Name:          brcm_patchram_plus.c
**
**  Description:   This program downloads a patchram files in the HCD format
**                 to Broadcom Bluetooth based silicon and combo chips and
**				   and other utility functions.
**
**                 It can be invoked from the command line in the form
**						<-d> to print a debug log
**						<--patchram patchram_file>
**						<--baudrate baud_rate>
**						<--bd_addr bd_address>
**						<--enable_lpm>
**						<--enable_hci>
**						<--use_baudrate_for_download>
**						<--scopcm=sco_routing,pcm_interface_rate,frame_type,
**							sync_mode,clock_mode,lsb_first,fill_bits,
**							fill_method,fill_num,right_justify>
**
**							Where
**
**							sco_routing is 0 for PCM, 1 for Transport,
**							2 for Codec and 3 for I2S,
**
**							pcm_interface_rate is 0 for 128KBps, 1 for
**							256 KBps, 2 for 512KBps, 3 for 1024KBps,
**							and 4 for 2048Kbps,
**
**							frame_type is 0 for short and 1 for long,
**
**							sync_mode is 0 for slave and 1 for master,
**
**							clock_mode is 0 for slabe and 1 for master,
**
**							lsb_first is 0 for false aand 1 for true,
**
**							fill_bits is the value in decimal for unused bits,
**
**							fill_method is 0 for 0's and 1 for 1's, 2 for
**								signed and 3 for programmable,
**
**							fill_num is the number or bits to fill,
**
**							right_justify is 0 for false and 1 for true
**
**						<--i2s=i2s_enable,is_master,sample_rate,clock_rate>
**
**							Where
**
**							i2s_enable is 0 for disable and 1 for enable,
**
**							is_master is 0 for slave and 1 for master,
**
**							sample_rate is 0 for 8KHz, 1 for 16Khz and
**								2 for 4 KHz,
**
**							clock_rate is 0 for 128KHz, 1 for 256KHz, 3 for
**								1024 KHz and 4 for 2048 KHz.
**
**						<--no2bytes skips waiting for two byte confirmation
**							before starting patchram download. Newer chips
**                          do not generate these two bytes.>
**						<--tosleep=number of microsseconds to sleep before
**							patchram download begins.>
**						uart_device_name
**
**                 For example:
**
**                 brcm_patchram_plus -d --patchram  \
**						BCM2045B2_002.002.011.0348.0349.hcd /dev/ttyHS0
**
**                 It will return 0 for success and a number greater than 0
**                 for any errors.
**
**                 For Android, this program invoked using a 
**                 "system(2)" call from the beginning of the bt_enable
**                 function inside the file 
**                 system/bluetooth/bluedroid/bluetooth.c.
**
**                 If the Android system property "ro.bt.bcm_bdaddr_path" is
**                 set, then the bd_addr will be read from this path.
**                 This is overridden by --bd_addr on the command line.
**  
******************************************************************************/

// TODO: Integrate BCM support into Bluez hciattach

#include <stdio.h>
#include <getopt.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdlib.h>

#ifdef ANDROID
#include <termios.h>
#else
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <limits.h>
#endif

#include <string.h>
#include <signal.h>

#ifdef ANDROID
#include <cutils/properties.h>
#define LOG_TAG "brcm_patchram_plus"
#include <cutils/log.h>
#undef printf
#define printf ALOGD
#undef fprintf
#define fprintf(x, ...) \
  { if(x==stderr) ALOGE(__VA_ARGS__); else fprintf(x, __VA_ARGS__); }

#endif //ANDROID

#ifndef N_HCI
#define N_HCI	15
#endif

#define HCIUARTSETPROTO		_IOW('U', 200, int)
#define HCIUARTGETPROTO		_IOR('U', 201, int)
#define HCIUARTGETDEVICE	_IOR('U', 202, int)

#define HCI_UART_H4		0
#define HCI_UART_BCSP	1
#define HCI_UART_3WIRE	2
#define HCI_UART_H4DS	3
#define HCI_UART_LL		4

typedef unsigned char uchar;

int uart_fd = -1;
int hcdfile_fd = -1;
int termios_baudrate = 0;
int bdaddr_flag = 0;
int enable_lpm = 0;
int enable_hci = 0;
int use_baudrate_for_download = 0;
int debug = 0;
int scopcm = 0;
int i2s = 0;
int no2bytes = 0;
int tosleep = 0;

struct termios termios;
uchar buffer[1024];

uchar hci_reset[] = { 0x01, 0x03, 0x0c, 0x00 };

uchar hci_rtksyc[] = { 0xc0, 0x00, 0x2f, 0x00,0xd0, 0x01,0x7e,0xc0};


void
init_uart_brcm()
{
	tcflush(uart_fd, TCIOFLUSH);
	int n = tcgetattr(uart_fd, &termios);
	fprintf(stderr, "tcgetattr %d\n",n);

#ifndef __CYGWIN__
	cfmakeraw(&termios);
#else
	termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                | INLCR | IGNCR | ICRNL | IXON);
	termios.c_oflag &= ~OPOST;
	termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	termios.c_cflag &= ~(CSIZE | PARENB);
	termios.c_cflag |= CS8;
#endif

	//termios.c_cflag |= CRTSCTS;
	tcsetattr(uart_fd, TCSANOW, &termios);
	tcflush(uart_fd, TCIOFLUSH);
	tcsetattr(uart_fd, TCSANOW, &termios);
	tcflush(uart_fd, TCIOFLUSH);
	tcflush(uart_fd, TCIOFLUSH);
	cfsetospeed(&termios, B115200);
	cfsetispeed(&termios, B115200);
	tcsetattr(uart_fd, TCSANOW, &termios);
}


void
init_uart_rtk()
{
	tcflush(uart_fd, TCIOFLUSH);
	int n = tcgetattr(uart_fd, &termios);
	fprintf(stderr, "tcgetattr %d\n",n);

#ifndef __CYGWIN__
	cfmakeraw(&termios);
#else
	termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                | INLCR | IGNCR | ICRNL | IXON);
	termios.c_oflag &= ~OPOST;
	termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	termios.c_cflag &= ~(CSIZE | PARENB);
	termios.c_cflag |= CS8;
#endif
		termios.c_cflag &= ~CRTSCTS;

	termios.c_cflag |= PARENB;

	//termios.c_cflag |= CRTSCTS;
	tcsetattr(uart_fd, TCSANOW, &termios);
	tcflush(uart_fd, TCIOFLUSH);
	tcsetattr(uart_fd, TCSANOW, &termios);
	tcflush(uart_fd, TCIOFLUSH);
	tcflush(uart_fd, TCIOFLUSH);
	cfsetospeed(&termios, B115200);
	cfsetispeed(&termios, B115200);
	tcsetattr(uart_fd, TCSANOW, &termios);
}

void
dump(uchar *out, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if (i && !(i % 16)) {
			fprintf(stderr, "\n");
		}

		fprintf(stderr, "%02x ", out[i]);
	}

	fprintf(stderr, "\n");
}

int  ttytestResult =-1;
#define READTTY_TIMEOUT  30//3s
int readttyLen(int fd,unsigned char *buffer,int len)
{
	int count;
	int i= 0;
	int timeout=0;
	while(len){
		count = read(fd,&buffer[i],1);
		if(count == 1){
			i += count;
			len -= count;
		}
		else{
			usleep(100000);//100ms
			timeout ++;
			//printf("timeout %d\n", timeout);
			if(timeout >= READTTY_TIMEOUT)
				return -1;
		}
	}
	return i;
}
void readBrcmTty(int fd, unsigned char  *buffer)
{
	int i=0;
	int count;
	int len;
	count = readttyLen(fd,buffer,3);
	printf("readBrcmTty count11 %d\n", count);
	if(count < 3)
		return;
	i += count;
	len = buffer[2];
	
	count = readttyLen(fd,&buffer[i],len);
	if(count<len)
		return;
	i += count;

	//if (debug)
	{

		printf("readBrcmTty received %d\n", i);
		dump(buffer, i);
	}
	
	ttytestResult = 0;
	printf("bt ttytest read_event succ\n");
}

void readRtkTty(int fd, unsigned char  *buffer)
{
	int i=0;
	int count;
	int len;
	
	count = readttyLen(fd,buffer,16);
	if(count < 16)
		return;
	i += count;

	//if (debug)
	{

		printf("received %d\n", i);
		dump(buffer, i);
	}

	ttytestResult = 0;
	printf("bt ttytest read_event succ\n");
}


void
hci_send_cmd(uchar *buf, int len)
{
	if (debug) {
		fprintf(stderr, "writing\n");
		dump(buf, len);
	}

	int writelen=write(uart_fd, buf, len);
	fprintf(stderr, "writelen %d\n",writelen);
}



void
expired(int sig)
{
	fprintf(stderr, "close(uart_fd)111\n");
	kill(getpid(), SIGKILL);

	hci_send_cmd(hci_reset, sizeof(hci_reset));

	alarm(4);
}


#define CONF_COMMENT '#'
#define CONF_DELIMITERS " =\n\r\t"
#define CONF_VALUES_DELIMITERS "=\n\r\t"
#define CONF_MAX_LINE_LEN 255



void get_tty_conf(const char *p_path,char *ttyPort)
{
    FILE    *p_file;
    char    *p_name;
    char    *p_value;
    char    line[CONF_MAX_LINE_LEN+1]; /* add 1 for \0 char */

    fprintf(stderr,"Attempt to load conf from %s", p_path);

    if ((p_file = fopen(p_path, "r")) != NULL)
    {
        /* read line by line */
        while (fgets(line, CONF_MAX_LINE_LEN+1, p_file) != NULL)
        {
            if (line[0] == CONF_COMMENT)
                continue;

            p_name = strtok(line, CONF_DELIMITERS);

            if (NULL == p_name)
            {
                continue;
            }

            p_value = strtok(NULL, CONF_DELIMITERS);

            if (NULL == p_value)
            {
                fprintf(stderr,"vnd_load_conf: missing value for name: %s", p_name);
                continue;
            }

            if (strcmp("UartPort", (const char *)p_name) == 0){
				fprintf(stderr,"get ttyPort %s", p_value);
				strcpy(ttyPort,p_value);
				fclose(p_file);
				return;
            }
			
        }

        fclose(p_file);
    }
    else
    {
        fprintf(stderr,"vnd_load_conf file >%s< not found", p_path);
    }
	strcpy(ttyPort,"/dev/ttyUSB3");
}


int test_rtktty()
{
	init_uart_rtk();
	hci_send_cmd(hci_rtksyc, sizeof(hci_rtksyc));
	readRtkTty(uart_fd, buffer);
	return ttytestResult;
}
int test_brcmtty()
{
	init_uart_brcm();
	hci_send_cmd(hci_reset, sizeof(hci_reset));
	readBrcmTty(uart_fd, buffer);
	return ttytestResult;
}
static void ttytestThread(void *param)
{
	char ttyPort[30]={0};
	get_tty_conf("/vendor/etc/bluetooth/bt_vendor.conf",ttyPort);
	fprintf(stderr, "port  %s\n", ttyPort);
	
	 if ((uart_fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1) {
			 fprintf(stderr, "port could not be opened, error %d\n", errno);
		 }
	int i;
	for(i=0;i<3;i++){
	 if(test_brcmtty()>=0)
	 	return;
	 if(test_rtktty()>=0)
	 	return;
	}

}

int bluetoothtty_test()
{
    int i;
	
	pthread_t thread_id;
	pthread_create(&thread_id, NULL,(void*)ttytestThread, NULL);
	for(i=10;i>0;i--){
		sleep(1);
		if(ttytestResult == 0)
			return 0;
	}
	return -1;	

}



int
main (int argc, char **argv)
{
   /*char ttyPort[30]={0};
   get_tty_conf("/vendor/etc/bluetooth/bt_vendor.conf",ttyPort);
   fprintf(stderr, "port  %s\n", ttyPort);

    if ((uart_fd = open(ttyPort, O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1) {
			fprintf(stderr, "port could not be opened, error %d\n", errno);
		}

	init_uart();

	//proc_reset();
	hci_send_cmd(hci_reset, sizeof(hci_reset));
	//read_event(uart_fd, buffer);
	read_tty(uart_fd, buffer);*/
	bluetoothtty_test();

	exit(1);

}
