#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>

#include "mbus.h"

#define MAX_RX_SIZE 255

#define SERIAL_DEVICE1 "/dev/ttyAMA0"
#define SERIAL_DEVICE2 "/dev/ttyUSB0"
#define SERIAL_DEVICE3 "/dev/ttyS0"

char device[50];
#define RECEIVE_VTIME 2 // 1/10 sekunden

int fd;
uint8_t mbus_tg[MAX_RX_SIZE];

int init_size = 536; // 532 x 0x55 bei 2400 baud 8N1 = 2,2 sek.
int init_pause = 240; // ms
unsigned char prim_adr = 0x00;
int mbus_bytes = 0;
unsigned char frame = 0x00;

void anleitung()
{
	printf("\n_________________________________\n");
	printf("1: Sende snd_nke\n");
	printf("2: Sende snd_ud\n");
	printf("3: Sende req_ud2\n");
	printf("4: Sende req_A6\n");
	printf("5: Sende req_A6+req_ud2\n");
	printf("6: Sende select_slave\n");
	printf("7: Sende set_prim_adr\n");
	printf("8: Sende snd_ud with frame\n");
	printf("9: Sende init_new\n");

	printf("+: Init + 10 chars\t\t:%d\n", init_size);
	printf("-: Init - 10 chars\n");

	printf("a: Pause + 10 ms\t\t:%d\n", init_pause);
	printf("y: Pause - 10 ms\n");

	printf("s: Prim.Adr + 1\t\t\t:0x%02X\n", prim_adr);
	printf("x: Prim.Adr - 1\n");

	printf("d: frame + 1\t\t\t:0x%02X\n", frame);
	printf("c: frame - 1\n");
	printf("q: Beenden\n");
	printf("_________________________________\n");

}

int set_interface_attribs(int speed, int parity, int bits)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0)
	{
		printf("error %d from tcgetattr: %s\n", errno, strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag &= ~CSIZE;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag |= bits;
	tty.c_cflag |= parity | CREAD | CLOCAL;
	/* CREAD            characters may be read */
	/* CLOCAL         ignore modem state, local connection */

	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	tty.c_iflag |= (INPCK/* | ISTRIP*/);
	//tty.c_iflag |= PARMRK; //(IGNPAR);

	tty.c_oflag = 0;
	tty.c_lflag = 0;

	tty.c_cc[VMIN] = 0;		// mindestens n Zeichen empfangen
	tty.c_cc[VTIME] = RECEIVE_VTIME;		// Wartezeit von 1/10 s = 0,2 s

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		printf("error %d from tcsetattr\n", errno);
		return -1;
	}
	return 0;
}

/*void set_blocking(int should_block)
 {
 struct termios tty;
 memset(&tty, 0, sizeof(tty));
 if (tcgetattr(fd, &tty) != 0)
 {
 printf("error %d from tggetattr: %s\n", errno, strerror(errno));
 return;
 }

 tty.c_cc[VMIN] = should_block ? 1 : 0;
 tty.c_cc[VTIME] = 20;            // 2 seconds read timeout

 if (tcsetattr(fd, TCSANOW, &tty) != 0)
 printf("error %d setting term attributes\n", errno);
 }*/

int init()
{
#define INIT_CHAR	0x55
#define INIT_BAUD	B2400

	size_t ret_out;
	int tx = 0;
	unsigned char seq[8];

	memset(seq, INIT_CHAR, sizeof(seq));

	// Umschalten auf 8E1
	printf("Port Modus 8N1\n");
	set_interface_attribs(INIT_BAUD, 0, CS8);

	printf("init(): Sende Aufwachsequenz\n");

	while (tx < init_size)
	{
		ret_out = write(fd, seq, sizeof(seq));
		if (ret_out == -1)
		{
			if (errno == EINTR)
				continue;

			printf("Error writing data: %d: %s\n", errno, strerror(errno));
			return -1;
		}
		tx += ret_out;
	}

	printf("Pause %d ms\n", init_pause);
	long pause_gesamt = ((int) (((float) init_size) * 4.1666)) + init_pause;

	int sekunden = pause_gesamt / 1000;
	unsigned long millisekunden = pause_gesamt - (sekunden * 1000);

	struct timespec sleeptime =
	{ .tv_sec = sekunden, .tv_nsec = millisekunden * 1000000 };

	if (nanosleep(&sleeptime, NULL) != 0)
		printf("Error in nanosleep: %d: %s\n", errno, strerror(errno));

	// Umschalten auf 8E1
	printf("Port Modus 8E1\n");
	set_interface_attribs(INIT_BAUD, PARENB, CS8);

//	printf("Leere rx buffer\n");
//	tcflush(fd, TCIFLUSH); /* Discards old data in the and rx buffer */

	return tx;
}

int snd_nke(unsigned char address)
{
	size_t ret_out;
	unsigned char seq[] =
	{ 0x10, 0x40, address, 0x00, 0x16 };

	// Checksumme berechnen
	for (int i = 1; i < sizeof(seq) - 2; i++)
		seq[sizeof(seq) - 2] += seq[i];
	for (int i = 0; i < sizeof(seq); i++)
		printf("[%02X]", seq[i]);
	printf("\n");

	printf("snd_nke(): Sende Initialisierung\n");

	ret_out = write(fd, seq, sizeof(seq));  // send snd_nke
	if (ret_out == -1)
	{
		if (errno == EINTR)
			snd_nke(address); // repeat write call
		else
		{
			printf("Error writing data: %d: %s\n", errno, strerror(errno));
			return -1;
		}
	}

	return ret_out;
}

int snd_ud(unsigned char address)
{
	size_t ret_out;
	unsigned char seq[] =
	{ 0x68, 0x03, 0x03, 0x68, 0x53, 0xFE, 0xA6, 0x00, 0x16 };

	// Checksumme berechnen
	for (int i = 4; i < sizeof(seq) - 2; i++)
		seq[sizeof(seq) - 2] += seq[i];

	printf("snd_ud(): Sende Anwenderdaten\n");
	for (int i = 0; i < sizeof(seq); i++)
		printf("[%02X]", seq[i]);
	printf("\n");

	ret_out = write(fd, seq, sizeof(seq));  // send snd_ud
	if (ret_out == -1)
	{
		if (errno == EINTR)
			snd_ud(address); // repeat write call
		else
		{
			printf("Error writing data: %d: %s\n", errno, strerror(errno));
			return -1;
		}
	}

	return ret_out;
}

int snd_ud_frame(unsigned char address, unsigned char frame)
{
	size_t ret_out;
	unsigned char seq[] =
	{ 0x68, 0x04, 0x04, 0x68, 0x53, address, 0x50, frame, 0x00, 0x16 };

	// Checksumme berechnen
	for (int i = 4; i < sizeof(seq) - 2; i++)
		seq[sizeof(seq) - 2] += seq[i];

	printf("snd_ud2(): Setze Datenrahmen: [%02X]\n", frame);
	for (int i = 0; i < sizeof(seq); i++)
		printf("[%02X]", seq[i]);
	printf("\n");

	ret_out = write(fd, seq, sizeof(seq));  // send snd_ud
	if (ret_out == -1)
	{
		if (errno == EINTR)
			snd_ud_frame(address, frame); // repeat write call
		else
		{
			printf("Error writing data: %d: %s\n", errno, strerror(errno));
			return -1;
		}
	}

	return ret_out;
}

int req_ud2(unsigned char address)
{
	size_t ret_out;
	unsigned char seq[] =
	{ 0x10, 0x5B, address, 0x00, 0x16 };

	// Checksumme berechnen
	for (int i = 1; i < sizeof(seq) - 2; i++)
		seq[sizeof(seq) - 2] += seq[i];

	printf("req_ud2(): Sende Anfrage Daten Klasse 2\n");
	for (int i = 0; i < sizeof(seq); i++)
		printf("[%02X]", seq[i]);
	printf("\n");

	ret_out = write(fd, seq, sizeof(seq));  // send req_ud2
	if (ret_out == -1)
	{
		if (errno == EINTR)
			req_ud2(address); // repeat write call
		else
		{
			printf("Error writing data: %d: %s\n", errno, strerror(errno));
			return -1;
		}
	}

	return ret_out;
}

int req_A6(unsigned char address)
{
	size_t ret_out;
	unsigned char seq[] =
	{ 0x68, 0x03, 0x03, 0x68, 0x53, address, 0xA6, 0x00, 0x16 };

	// Checksumme berechnen
	for (int i = 4; i < sizeof(seq) - 2; i++)
		seq[sizeof(seq) - 2] += seq[i];

	printf("req_A6(): Sende Anfrage Daten\n");
	for (int i = 0; i < sizeof(seq); i++)
		printf("[%02X]", seq[i]);
	printf("\n");

	ret_out = write(fd, seq, sizeof(seq));
	if (ret_out == -1)
	{
		if (errno == EINTR)
			req_A6(address); // repeat write call
		else
		{
			printf("Error writing data: %d: %s\n", errno, strerror(errno));
			return -1;
		}
	}

	return ret_out;
}

int init_new(unsigned char address)
{
	size_t ret_out;
	unsigned char seq[] =
	{ 0x68, 0x04, 0x04, 0x68, 0x53, 0xF9, 0x50, 0x01, 0x0A, 0x67, 0xFC };

	// Checksumme berechnen
	/*	for (int i = 4; i < sizeof(seq) - 2; i++)
	 seq[sizeof(seq) - 2] += seq[i];
	 */
	printf("init_new(): Sende Anfrage Daten\n");
	for (int i = 0; i < sizeof(seq); i++)
		printf("[%02X]", seq[i]);
	printf("\n");

	ret_out = write(fd, seq, sizeof(seq));
	if (ret_out == -1)
	{
		if (errno == EINTR)
			init_new(address); // repeat write call
		else
		{
			printf("Error writing data: %d: %s\n", errno, strerror(errno));
			return -1;
		}
	}

	return ret_out;
}

int select_slave()
{
	/*
	 68 0B 0B 68 53 FD 52 NN NN NN NN HH HH ID MM CS 16
	 Response: E5 (only if filter matches)
	 Structure of filter:
	 4-byte BCD NN (serial number) $F digit joker
	 2-byte HST HH (manufacturer code) $FF byte joker
	 1-byte ID (SHARKY 775: $2F) ID (identification code) $FF joker
	 1-byte SMED MM (medium code) $FF joker
	 */
#define NN1	0x17
#define NN2	0x42
#define NN3	0x52
#define NN4	0x11
#define HH1	0xFF
#define HH2	0xFF
#define ID		0xFF
#define MM	0xFF

	size_t ret_out;
	unsigned char seq[] =
	{ 0x68, 0x0B, 0x0B, 0x68, 0x53, 0xFD, 0x52, NN1, NN2, NN3, NN4, HH1, HH2, ID, MM, 0x00, 0x16 };

	// Checksumme berechnen
	for (int i = 4; i < sizeof(seq) - 2; i++)
	{
		seq[sizeof(seq) - 2] += seq[i];
	}

	printf("select_slave():\n");
	for (int i = 0; i < sizeof(seq); i++)
		printf("[%02X]", seq[i]);
	printf("\n");

	ret_out = write(fd, seq, sizeof(seq));
	if (ret_out == -1)
	{
		if (errno == EINTR)
			select_slave(); // repeat write call
		else
		{
			printf("Error writing data: %d: %s\n", errno, strerror(errno));
			return -1;
		}
	}

	return ret_out;
}

int set_prim_adr(unsigned char address)
{
#define DIF	0x01
#define VIF		0x7A
	size_t ret_out;
	unsigned char seq[] =
	{ 0x68, 0x06, 0x06, 0x68, 0x73, 0xFE, 0x51, DIF, VIF, address, 0x00, 0x16 };

	// Checksumme berechnen
	for (int i = 4; i < sizeof(seq) - 2; i++)
		seq[sizeof(seq) - 2] += seq[i];

	printf("set_prim_adr():\n");
	for (int i = 0; i < sizeof(seq); i++)
		printf("[%02X]", seq[i]);
	printf("\n");

	ret_out = write(fd, seq, sizeof(seq));
	if (ret_out == -1)
	{
		if (errno == EINTR)
			set_prim_adr(address); // repeat write call
		else
		{
			printf("Error writing data: %d: %s\n", errno, strerror(errno));
			return -1;
		}
	}

	return ret_out;
}

int listen(unsigned char* data, int lenght, int max_lenght)
{
	int mbus_bytes_received = 0, data_pos = 0, i;
	unsigned char rxbuffer[MAX_RX_SIZE];
	int timeout = 2;
	/*	struct timespec delay =
	 { .tv_sec = 0, .tv_nsec = 10000000 }; // 10 ms
	 */
	while (1)
	{
		mbus_bytes_received = read(fd, rxbuffer, MAX_RX_SIZE);
		if (mbus_bytes_received == -1)
		{
			printf("error %d on reading data from serial device\n", errno);
			break;
		}

		if (mbus_bytes_received == 0)
		{
			if (--timeout)
				continue;
			else
				break;
		}

		printf("\nmbus_bytes_received: %d: ", mbus_bytes_received);
		for (int i = 0; i < mbus_bytes_received; i++)
		{
			printf("[%02X] ", rxbuffer[i]);
		}
		printf("\n");

		for (i = 0; i < mbus_bytes_received; i++)
		{
			if (data_pos < max_lenght)
				data[data_pos++] = rxbuffer[i];
			else
				return data_pos;

			if (data_pos == lenght)
				return data_pos;
		}
	}

	return data_pos;
}

void transmit(char c)
{
	int i = init();
	printf("init(): %d mbus_bytes written\n", i);

	switch (c)
	{
		case '1':
			i = snd_nke(prim_adr);
			printf("snd_nke(): %d mbus_bytes written\n", i);
			mbus_bytes = listen(mbus_tg, 1, MAX_RX_SIZE);
			break;
		case '2':
			i = snd_ud(prim_adr);
			printf("snd_ud(): %d mbus_bytes written\n", i);
			mbus_bytes = listen(mbus_tg, MAX_RX_SIZE, MAX_RX_SIZE);
			break;
		case '3':
			i = req_ud2(prim_adr);
			printf("req_ud2(): %d mbus_bytes written\n", i);
			mbus_bytes = listen(mbus_tg, MAX_RX_SIZE, MAX_RX_SIZE);
			break;
		case '4':
			i = req_A6(prim_adr);
			printf("req_A6(): %d mbus_bytes written\n", i);
			mbus_bytes = listen(mbus_tg, MAX_RX_SIZE, MAX_RX_SIZE);
			break;
		case '5':
			i = req_A6(prim_adr);
			printf("req_A6(): %d mbus_bytes written\n", i);
			mbus_bytes = listen(mbus_tg, MAX_RX_SIZE, MAX_RX_SIZE);
			printf("mbus_bytes: %d\n", mbus_bytes);

			struct timespec sleeptime =
			{ .tv_sec = 1, .tv_nsec = 0 };

			if (nanosleep(&sleeptime, NULL) != 0)
				printf("Error in nanosleep: %d: %s\n", errno, strerror(errno));

			mbus_bytes = 0;
			i = req_ud2(prim_adr);
			printf("req_ud2(): %d mbus_bytes written\n", i);
			mbus_bytes = listen(mbus_tg, MAX_RX_SIZE, MAX_RX_SIZE);
			break;
		case '6':
			i = select_slave();
			printf("select_slave(): %d mbus_bytes written\n", i);
			mbus_bytes = listen(mbus_tg, MAX_RX_SIZE, MAX_RX_SIZE);
			break;
		case '7':
			i = set_prim_adr(prim_adr);
			printf("set_prim_adr(): %d mbus_bytes written\n", i);
			mbus_bytes = listen(mbus_tg, MAX_RX_SIZE, MAX_RX_SIZE);
			break;
		case '8':
			i = snd_ud_frame(prim_adr, frame);
			printf("snd_ud_frame(): %d mbus_bytes written\n", i);
			mbus_bytes = listen(mbus_tg, MAX_RX_SIZE, MAX_RX_SIZE);
			break;
		case '9':
			i = init_new(prim_adr);
			printf("init_new(): %d mbus_bytes written\n", i);
			mbus_bytes = listen(mbus_tg, MAX_RX_SIZE, MAX_RX_SIZE);
			break;
	}

	printf("mbus_bytes: %d\n", mbus_bytes);
}

int main(int argc, char *argv[])
{
	if (sizeof(size_t) == 4)
		printf("32 bit system\n");
	else if (sizeof(size_t) == 8)
		printf("64 bit system\n");

	printf("Size uint8_t=\t%lu\n", sizeof(uint8_t));
	printf("Size uint16_t=\t%lu\n", sizeof(uint16_t));
	printf("Size uint32_t=\t%lu\n", sizeof(uint32_t));
	printf("Size uint64_t=\t%lu\n", sizeof(uint64_t));
	printf("Size int=\t%lu\n", sizeof(int));
	printf("Size long=\t%lu\n", sizeof(long));
	printf("Size long long=\t%lu\n", sizeof(long long));

	struct termios old_tio, new_tio;
	unsigned char c;
	int error;
	allmess_zaehler zaehler;


	/*uint8_t tg[] = {0x68, 0x4D, 0x4D, 0x68, 0x08, 0x00, 0x72, 0x32, 0x00, 0x54, 0x17, 0x92, 0x26, 0x17, 0x04, 0x01, 0x00, 0x00, 0x00, 0x0C, 0x78, 0x32, 0x00, 0x54, 0x17, 0x04, 0x06, 0x16, 0x21, 0x00, 0x00, 0x0C, 0x14, 0x88, 0x09, 0x23, 0x00, 0x0B, 0x2D, 0x08, 0x00, 0x00, 0x0B, 0x3B, 0x39, 0x03, 0x00, 0x0A, 0x5A, 0x78, 0x02, 0x0A, 0x5E, 0x57, 0x02, 0x0B, 0x61, 0x11, 0x02, 0x00, 0x04, 0x6D, 0x06, 0x0C, 0x84, 0x21, 0x02, 0x27, 0x07, 0x04, 0x09, 0xFD, 0x0E, 0x07, 0x09, 0xFD, 0x0F, 0x11, 0x0F, 0x00, 0x00, 0xAD, 0x16};
	mbus_bytes = sizeof (tg);
	mbus_parse_telegram(tg, mbus_bytes, &zaehler);
	mbus_print(&zaehler);
	return 0;
        */

	/* get the terminal settings for stdin */
	tcgetattr(STDIN_FILENO, &old_tio);

	/* we want to keep the old setting to restore them a the end */
	new_tio = old_tio;

	/* disable canonical mode (buffered i/o) and local echo */
	new_tio.c_lflag &= (~ICANON & ~ECHO);

	/* set the new settings immediately */
	tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

	if (argc > 1)
		strncpy(device, argv[1], 50);
	else
		strncpy(device, SERIAL_DEVICE1, 50);

	fd = open(device, O_RDWR | O_NOCTTY/* | O_SYNC | O_NDELAY | O_NONBLOCK*/);
	if (fd == -1)
	{
		strncpy(device, SERIAL_DEVICE2, 50);
		fd = open(device, O_RDWR | O_NOCTTY/* | O_SYNC | O_NDELAY | O_NONBLOCK*/);
		if (fd == -1)
		{
			strncpy(device, SERIAL_DEVICE3, 50);
			fd = open(device, O_RDWR | O_NOCTTY/* | O_SYNC | O_NDELAY | O_NONBLOCK*/);
			if (fd == -1)
			{
				printf("error %d from open(): %s\n", errno, strerror(errno));
				tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
				return -1;
			}
		}
	}

	printf("Using serial device: %s\n", device);

	anleitung();

	while (1)
	{
		tcflush(fd, TCIOFLUSH); /* Discards old data in the tx and rx buffer */
		mbus_bytes = 0;
		memset(&mbus_tg, 0, sizeof(mbus_tg));

		c = getchar();

		if (c == 'q')
			break;

		if (c == 'h' || c == ' ')
		{
			anleitung();
			continue;
		}

		if (c == '+')
		{
			init_size += 10;
			printf("init_size = %d, time: %d ms\n", init_size, (int) (((float) init_size) * 4.1666));
			continue;
		}
		else if (c == '-')
		{
			if (init_size > 10)
				init_size -= 10;
			printf("init_size = %d, time: %d ms\n", init_size, (int) (((float) init_size) * 4.1666));
			continue;
		}
		else if (c == 'a')
		{
			init_pause += 10;
			printf("init_pause = %d ms\n", init_pause);
			continue;
		}
		else if (c == 'y')
		{
			if (init_pause > 10)
				init_pause -= 10;
			printf("init_pause = %d ms\n", init_pause);
			continue;
		}
		else if (c == 's')
		{
			prim_adr += 1;
			printf("prim_adr = 0x%02X\n", prim_adr);
			continue;
		}
		else if (c == 'x')
		{
			prim_adr -= 1;
			printf("prim_adr = 0x%02X\n", prim_adr);
			continue;
		}
		else if (c == 'd')
		{
			frame += 1;
			printf("frame = 0x%02X\n", frame);
			continue;
		}
		else if (c == 'c')
		{
			frame -= 1;
			printf("frame = 0x%02X\n", frame);
			continue;
		}

		if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'x'))
		{
			transmit(c);

			if (mbus_bytes > 0)
			{
				error = mbus_parse_telegram(mbus_tg, mbus_bytes, &zaehler);
				if (error < 0)
					printf("Error: %d\n", error);
				else {
					mbus_print(&zaehler);
					fflush(stdout);
				}
			}
		}

	}

	close(fd);
	printf("\n");
	/* restore the former settings */
	tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);

	return 0;
}

