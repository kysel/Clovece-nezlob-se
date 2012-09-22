#include<avr/io.h>
#include<avr/interrupt.h>
#include<math.h>
#include<avr/pgmspace.h>

#ifndef __cplusplus
#error Nezapomente si v nastaveni AVR studiu zapnout "-x c++" !
#endif

#ifndef DYNAMIXEL_F_CPU
#define DYNAMIXEL_F_CPU 16000000
#endif

#ifndef F_CPU
#define F_CPU DYNAMIXEL_F_CPU
#endif

#if defined(F_CPU) && F_CPU != DYNAMIXEL_F_CPU
#error Mate v nastaveni spatne nastavenou frekvenci procesoru !
#endif

#ifndef DYNAMIXEL_BUS_RX_BUF
#define DYNAMIXEL_BUS_RX_BUF 96
#endif

#ifndef DYNAMIXEL_BUS_TX_BUF
#define DYNAMIXEL_BUS_TX_BUF 96
#endif

#ifndef DYNAMIXEL_BUS_BPS
#define DYNAMIXEL_BUS_BPS 1000000
#endif

#ifndef DYNAMIXEL_PC_RX_BUF
#define DYNAMIXEL_PC_RX_BUF 96
#endif

#ifndef DYNAMIXEL_PC_TX_BUF
#define DYNAMIXEL_PC_TX_BUF 96
#endif

#ifndef DYNAMIXEL_PC_BPS
#define DYNAMIXEL_PC_BPS 57600
#endif

#ifndef DYNAMIXEL_RETURN
#define DYNAMIXEL_RETURN 2
#endif

#ifndef endl
#define endl "\n\r"
#endif

#ifndef DYNAMIXEL_RESPOND_TIME
#define DYNAMIXEL_RESPOND_TIME 500
#endif

#ifndef AX_ERROR_BUF
#define AX_ERROR_BUF 96
#endif

#ifndef EEPROM_FIXPOINT
#define EEPROM_FIXPOINT 32768.0
#endif

#ifndef STRING_DEFAULT_SIZE
#define STRING_DEFAULT_SIZE 32
#endif 

#ifndef GETNUMBER_FAILURE_RETURN_VALUE
#define GETNUMBER_FAILURE_RETURN_VALUE -2147483647
#endif

#define P_MODEL_NUMBER_L 0
#define P_MODOEL_NUMBER_H 1
#define P_VERSION 2
#define P_ID 3
#define P_BAUD_RATE 4
#define P_RETURN_DELAY_TIME 5
#define P_CW_ANGLE_LIMIT_L 6
#define P_CW_ANGLE_LIMIT_H 7
#define P_CCW_ANGLE_LIMIT_L 8
#define P_CCW_ANGLE_LIMIT_H 9
#define P_SYSTEM_DATA2 10
#define P_LIMIT_TEMPERATURE 11
#define P_DOWN_LIMIT_VOLTAGE 12
#define P_UP_LIMIT_VOLTAGE 13
#define P_MAX_TORQUE_L 14
#define P_MAX_TORQUE_H 15
#define P_RETURN_LEVEL 16
#define P_ALARM_LED 17
#define P_ALARM_SHUTDOWN 18
#define P_OPERATING_MODE 19
#define P_DOWN_CALIBRATION_L 20
#define P_DOWN_CALIBRATION_H 21
#define P_UP_CALIBRATION_L 22
#define P_UP_CALIBRATION_H 23
#define P_TORQUE_ENABLE (24)
#define P_LED (25)
#define P_CW_COMPLIANCE_MARGIN (26)
#define P_CCW_COMPLIANCE_MARGIN (27)
#define P_CW_COMPLIANCE_SLOPE (28)
#define P_CCW_COMPLIANCE_SLOPE (29)
#define P_GOAL_POSITION_L (30)
#define P_GOAL_POSITION_H (31)
#define P_GOAL_SPEED_L (32)
#define P_GOAL_SPEED_H (33)
#define P_TORQUE_LIMIT_L (34)
#define P_TORQUE_LIMIT_H (35)
#define P_PRESENT_POSITION_L (36)
#define P_PRESENT_POSITION_H (37)
#define P_PRESENT_SPEED_L (38)
#define P_PRESENT_SPEED_H (39)
#define P_PRESENT_LOAD_L (40)
#define P_PRESENT_LOAD_H (41)
#define P_PRESENT_VOLTAGE (42)
#define P_PRESENT_TEMPERATURE (43)
#define P_REGISTERED_INSTRUCTION (44)
#define P_PAUSE_TIME (45)
#define P_MOVING (46)
#define P_LOCK (47)
#define P_PUNCH_L (48)
#define P_PUNCH_H (49)
//sensor
#define P_OBSTACLE_DETECTED_COMPARE_VALUE (20)
#define P_LIGHT_DETECTED_COMPARE_VALUE (21)
#define P_LEFT_IR_SENSOR_DATA (26)
#define P_CENTER_IR_SENSOR_DATA (27)
#define P_RIGHT_IR_SENSOR_DATA (28)
#define P_LEFT_LUMINOSITY (29)
#define P_CENTER_LUMINOSITY (30)
#define P_RIGHT_LUMINOSITY (31)
#define P_OBSTACLE_DETECTION_FLAG (32)
#define P_LUMINOSITY_DETECTION_FLAG (33)
#define P_SOUND_DATA (35)
#define P_SOUND_DATA_MAX_HOLD (36)
#define P_SOUND_DETECTED_COUNT (37)
#define P_SOUND_DETECTED_TIME_L (38)
#define P_SOUND_DETECTED_TIME_H (39)
#define P_BUZZER_INDEX (40)
#define P_BUZZER_TIME (41)
#define P_IR_REMOCON_ARRIVED (46)
#define P_IR_REMOCON_RX_DATA_0 (48)
#define P_IR_REMOCON_RX_DATA_1 (49)
#define P_IR_REMOCON_TX_DATA_0 (50)
#define P_IR_REMOCON_TX_DATA_1 (51)
#define P_OBSTACLE_DETECTED_COMPARE (52)
#define P_LIGHT_DETECTED_COMPARE (53)
//--- Instruction ---
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_ACTION 0x05
#define INST_RESET 0x06
#define INST_DIGITAL_RESET 0x07
#define INST_SYSTEM_READ 0x0C
#define INST_SYSTEM_WRITE 0x0D
#define INST_SYNC_WRITE 0x83
#define INST_SYNC_REG_WRITE 0x84

#define BROADCASTING_ID 0xFE


#define DYNAMIXEL_BOOTLOADER_SEQ() \
	asm volatile ( \
		"ldi r31, 0xF0\n" \
		"ldi r30, 0x00\n" \
		"ijmp" \
		);

#define RESET_SEQ() \
	asm volatile ( \
		"ldi r31, 0x0\n" \
		"ldi r30, 0x0\n" \
		"ijmp" \
		);

template <typename T, uint16_t size>
class queue {
	volatile T m [size];
	volatile uint16_t m_read;
	volatile uint16_t m_write;
public:
	volatile bool overwrite;
	bool is_empty() const
	{
		if(m_write == m_read)
			return true;
		return false;
	}
	bool is_full() const
	{
		if((m_write == (m_read - 1))||((m_write == (size - 1))&&(m_read == 0)))
			return true;
		return false;
	}
	bool push (T data)
	{
		m[m_write] = data;
		if(is_full()&&!overwrite)
			return false;
		if(++m_write == size)
			m_write = 0;
		return true;
	}
	T pop ()
	{
		if(is_empty())
			return m[m_read];
		T data = m[m_read];
		if(++m_read == size)
			m_read = 0;
		return data;
	}
	T operator[](uint16_t index) const
	{
		return m[index];
	}
	inline void clean() const
	{
		m_read = 0;
		m_write = 0;
	}
	void copy(T *data)
	{
		for(uint16_t i = 0; i < size; ++i)
		{
			*(data + i) = m[i];
		}
	}
	void format(T data = 0)
	{
		for(uint16_t i = 0; i < size; ++i)
			m[i] = data;
		clean();
	}
	inline int state() const
	{
		return m_read - m_write;
	}
};

//template <uint8_t m_size = STRING_DEFAULT_SIZE>
class string {
	const static uint8_t m_size = STRING_DEFAULT_SIZE;
	char m_data[STRING_DEFAULT_SIZE];
	uint8_t pointer;
public:
	string()
	{
		//m_size = STRING_DEFAULT_SIZE;
		for(pointer = 0; pointer != m_size; ++pointer)
			m_data[pointer] = 0;
		pointer = 0;
	}
	char operator[](const uint8_t &index) const
	{
		if(index < m_size)
			return m_data[index];
		return m_data[m_size-1];
	}

	char & operator[](const uint8_t &index)
	{
		if(index > m_size)
			return m_data[m_size-1];
		pointer = index;
		return m_data[index];
	}
	uint8_t size() const
	{
		return m_size;
	}
	uint8_t getPointer() const
	{
		return pointer;
	}
	void operator = (const char *data)
	{
		for(pointer = 0; pointer != (m_size - 1); ++pointer)
		{
			*(m_data+pointer) = *(data+pointer);
			if(*(data+pointer) == 0)
				return;
		}
		*(m_data+m_size-1) = 0;
	}
	bool operator == (const char *data) const
	{
		for(uint8_t i = 0; i != m_size; ++i)
		{
			if(*(m_data+i) != *(data+i))
				return false;
			if(*(m_data+i) == 0)
				return true;
		}
		return true;
	}
	void operator --()
	{
		if(pointer != 0)
			--pointer;
		m_data[pointer] = 0;
	}
	void operator += (const char & ch)
	{
		m_data[pointer] = ch;
		if(pointer != m_size)
			++pointer;
	}
	/*void operator += (const char *data)
	{
		for(uint8_t i = 0; (*(data+i) != 0) && (pointer != m_size); ++i)
		{
			m_data[pointer] = *(data+i);
		}
	}*/
	void clear(const char ch = 0)
	{
		for(pointer = 0; pointer != m_size; ++pointer)
			m_data[pointer] = ch;
		pointer = 0;
	}
};

int32_t abs(int32_t n)
{
	return n<0?-n:n;
}

class USART1_t {

	queue <char, DYNAMIXEL_PC_RX_BUF> m_rx;
	queue <char, DYNAMIXEL_PC_RX_BUF> m_tx;
	uint8_t numbersSpaces;
	bool cin_isFirst;
	string cin_buf;
	bool cin_isGood;

public:

	void data_in(char &ch)
	{
		m_rx.push(ch);
	}

	bool data_out(char &ch)
	{
		if(m_tx.is_empty())
			return false;
		ch = m_tx.pop();
		return true;
	}

	bool is_send() const
	{
		if(!m_tx.is_empty() && ((UCSR1A & (1<<TXC1)) == 0))
			return false;
		UCSR1A |= (1<<TXC1);
		return true;
	}

	void init(uint32_t speed)
	{
		m_tx.overwrite = false;
		m_rx.overwrite = false;
		UCSR1A = (1<<U2X1);
		UCSR1B = ((1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1));
		UCSR1C = ((1<<UCSZ10)|(1<<UCSZ11));
		speed = (((F_CPU/(float(8*speed)))-1)-((F_CPU/(8*speed))-1))<0.5?((F_CPU/(8*speed))-1):((F_CPU/(8*speed)));
		UBRR1H = ((speed&0xFF00)>>8);
		UBRR1L = (speed&0x00FF);
		numbersSpaces = 1;
		cin_isFirst = true;
		cin_isGood = true;
	}
	void sendChar(char data)
	{
		while(!m_tx.push(data)) {}
		UCSR1B |= (1<<UDRIE1);
	}
	
	void sendCharImediatly(char data)
	{
		UCSR1A |= (1<<TXC1);
		UDR1 = data;
		while((UCSR1A & (1<<TXC1)) == 0){}
	}

	void send(const char * str)
	{
		for(; *str != 0; ++str)
			sendChar(*str);
	}
	
	void sendImediatly(const char * str)
	{
		for(; *str != 0; ++str)
			sendCharImediatly(*str);
	}

	void sendNumber(int32_t number, uint8_t width = 4)
	{
		char buff[12];
		uint8_t buffP = 0;
		if(number == 0)
		{
			for(uint8_t i = 1; i < width; ++i)
				sendChar(' ');
			sendChar('0');
			return;
		}
		else if(number < 0)
		{
			buff[buffP++] = '-';
			number = -number;
			--width;
		}
		for(int32_t i = 1000000000;i >= 1; i = (i / 10))
		{
			if(number >= i)
			{
				buff[buffP++] = ('0' + ((number / i) % 10));
				if(width != 0)
					--width;
			}
		}
		for(uint8_t i = 0; i < width; ++i)
			sendChar(' ');
		for(uint8_t i = 0; i < buffP; ++i)
			sendChar(buff[i]);
	}

	void sendNumberImediatly(int32_t number, uint8_t width = 4)
	{
		char buff[12];
		uint8_t buffP = 0;
		if(number == 0)
		{
			for(uint8_t i = 1; i < width; ++i)
				sendCharImediatly(' ');
			sendCharImediatly('0');
			return;
		}
		else if(number < 0)
		{
			buff[buffP++] = '-';
			number = -number;
			--width;
		}
		for(int32_t i = 1000000000;i >= 1; i = (i / 10))
		{
			if(number >= i)
			{
				buff[buffP++] = ('0' + ((number / i) % 10));
				if(width != 0)
					--width;
			}
		}
		for(uint8_t i = 0; i < width; ++i)
			sendCharImediatly(' ');
		for(uint8_t i = 0; i < buffP; ++i)
			sendCharImediatly(buff[i]);
	}

	void sendHexNumber(int32_t number)
	{
		if(number == 0)
		{
			send("0x00");
			return;
		}
		else if(number < 0)
		{
			sendChar('-');
			number = -number;
		}
		send("0x");
		uint8_t zero = 0;
		for(int32_t i = 268435456; i >= 1; i = (i / 16))
		{
			if(number >= i)
			{
				if((zero % 2) == 1)
				{
					send("0");
					zero = 0;
				}
				uint8_t lastChar = ((number / i) % 16);
				if(lastChar < 10)
					sendChar('0' + lastChar);
				else
					sendChar('A' + lastChar - 10);
			}
			else
				++zero;
		}
	}

	void sendHexNumberImediatly(int32_t number)
	{
		if(number == 0)
		{
			sendImediatly("0x00");
			return;
		}
		else if(number < 0)
		{
			sendCharImediatly('-');
			number = -number;
		}
		sendImediatly("0x");
		uint8_t zero = 0;
		for(int32_t i = 268435456; i >= 1; i = (i / 16))
		{
			if(number >= i)
			{
				if((zero % 2) == 1)
				{
					send("0");
					zero = 0;
				}
				uint8_t lastChar = ((number / i) % 16);
				if(lastChar < 10)
					sendCharImediatly('0' + lastChar);
				else
					sendCharImediatly('A' + lastChar - 10);
			}
			else
				++zero;
		}
	}

	void sendNumbers(uint8_t *n, uint16_t length)
	{
		for(uint16_t i = 0; i < length; ++i)
		{
			sendNumber(*(n + i));
			send("; ");
		}
		send("\n\r");
	}

	void wait()
	{
		while(!is_send()) {}
	}

	bool peek(char & data)
	{
		if(m_rx.is_empty())
			return false;
		data = m_rx.pop();
		return true;
	}

	char get()
	{
		char data;
		while(!peek(data)) {}
		return data;
	}

//*********************************************************************************
	void setAlign(uint8_t n)
	{
		numbersSpaces = n>0?n:1;
	}
//********
	USART1_t &operator << (const char &ch)
	{
		sendChar(ch);
		return *this;
	}
	USART1_t &operator << (const char *ch)
	{
		send(ch);
		return *this;
	}
/*	USART1_t &operator << (const int &n)
	{
		sendNumber(n, numbersSpaces);
		return *this;
	}
	USART1_t &operator << (const unsigned int &n)
	{
		sendNumber(n, numbersSpaces);
		return *this;
	}*/
	USART1_t &operator << (const int8_t &n)
	{
		sendNumber(n, numbersSpaces);
		return *this;
	}
	USART1_t &operator << (const uint8_t &n)
	{
		sendNumber(n, numbersSpaces);
		return *this;
	}
	USART1_t &operator << (const int16_t &n)
	{
		sendNumber(n, numbersSpaces);
		return *this;
	}
	USART1_t &operator << (const uint16_t &n)
	{
		sendNumber(n, numbersSpaces);
		return *this;
	}
	USART1_t &operator << (const int32_t &n)
	{
		sendNumber(n, numbersSpaces);
		return *this;
	}
	USART1_t &operator << (const uint32_t &n)
	{
		sendNumber(n, numbersSpaces);
		return *this;
	}
	USART1_t &operator << (const bool &n)
	{
		send(n?"true":"false");
		return *this;
	}
	USART1_t &operator << (const string &s)
	{
		for(uint8_t i = 0; s[i] != 0; ++i)
			sendChar(s[i]);
		return *this;
	}
//input
	USART1_t &operator >> (char &ch)
	{
		uint8_t i = 0;
		char temp = get();
		while(temp != '\r')
		{
			if(i == 0)
				ch = temp;
			if(temp != 8)
				++i;
			else
				if(i != 0)
					--i;
		}
		return *this;
	}
	USART1_t &operator >> (string &s)
	{
		if(cin_isFirst)
		{
			char ch = get();
			cin_buf.clear();
			while(ch != '\r')
			{
				if(ch == 8)
				{
					if(cin_buf.getPointer() != 0)
						sendChar(ch);
					--cin_buf;
				}
				else
				{
					cin_buf += ch;
					if(cin_buf.getPointer() == cin_buf.size())
						sendChar('\a');
					else
						sendChar(ch);
				}
				ch = get();
			}
			send(endl);
			cin_isFirst = false;
		}
		uint8_t i;
		for(i = 0; (cin_buf[i] != ' ') && (cin_buf[i] != 0); ++i)
			s[i] = cin_buf[i];
		if(cin_buf[i++] == 0)
			cin_isFirst = true;
		else
		{
			uint8_t j;
			for(j = 0; cin_buf[i+j] != 0; ++j)
				cin_buf[j] = cin_buf[i+j];
			cin_buf[j] = 0;
		}
		s[i] = 0;
		return *this;
	}
	/*int32_t getNumber(const string &s)
	{
		int32_t n = 0;
		uint32_t rank = 1;
		for(int8_t p = s.getPointer()-2;p != -1; --p)
		{
			if(s[p]>='0' && s[p]<='9')
				n += rank * (s[p]-'0');
			else if(p == 0 && s[p] == '-')
				n = -n;
			else
				return GETNUMBER_FAILURE_RETURN_VALUE;//-(1<<31)+1
			rank *= 10;
		}
		return n;
	}*/
	template <typename T>
	bool getNumber(const string &s, T &n)
	{
		bool isNegative = s[0]=='-';
		n = 0;
		for(uint8_t i = isNegative?1:0; s[i] != 0; ++i)
		{
			if(s[i]<'0' || s[i]>'9')
				return false;
			n = n*10+s[i]-'0';
		}
		if(isNegative)
			n = -n;
		return true;
	}
	USART1_t &operator >> (uint8_t & n)
	{
		string s;
		if(!(*this>>s)||!getNumber(s, n))


			cin_isGood = false;


		return *this;
	}
	USART1_t &operator >> (int8_t & n)
	{
		string s;
		if(!(*this>>s)||!getNumber(s, n))


			cin_isGood = false;


		return *this;
	}
	USART1_t &operator >> (uint16_t & n)
	{
		string s;
		if(!(*this>>s)||!getNumber(s, n))


			cin_isGood = false;


		return *this;
	}
	USART1_t &operator >> (int16_t & n)
	{
		string s;
		if(!(*this>>s)||!getNumber(s, n))


			cin_isGood = false;


		return *this;
	}
	USART1_t &operator >> (uint32_t & n)
	{
		string s;
		if(!(*this>>s)||!getNumber(s, n))


			cin_isGood = false;


		return *this;
	}
	USART1_t &operator >> (int32_t & n)
	{
		string s;
		if(!(*this>>s)||!getNumber(s, n))


			cin_isGood = false;


		return *this;
	}
	operator void const *() const
	{
		return cin_isGood?this:0;
	}
	void clear()
	{
		cin_isGood = true;
	}
}; USART1_t pc;

ISR(USART1_RX_vect)
{
	char data = UDR1;
	if((UCSR1A & (1<<FE1)) == (1<<FE1))
		return;
	pc.data_in(data);
	static const unsigned char bootSeq[] = { '#', '#', '#', '#' };
	static uint8_t state = 0;
	if (data == bootSeq[state])
	{
		if (++state == 4)
		{
#ifndef DYNAMIXEL_BOOTLOADER_WDT
			UCSR1A = 0;
			UCSR1B = 0;
			UCSR1C = 0;
			UBRR1L = 0;
			UBRR1H = 0;
			DYNAMIXEL_BOOTLOADER_SEQ();
#else
			WDTCR |= (1<<WDE);
#endif
		}
	}
	else
		state = 0;
}
ISR(USART1_UDRE_vect)
{
	char data;
	if(pc.data_out(data))
		UDR1 = data;
	else
		UCSR1B &= ~(1<<UDRIE1);
}

/*
Cekani
Funguje od 2 do 4294967294 us
Zadava se v us pro F_CPU 16000000
*/ 
void syncWait(uint32_t time)
{
	time -= 2;
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
	for(uint32_t i = 0; i < time; ++i)
	{
		for(uint8_t j = 0; j < 5; ++j)
		{
			__asm__ volatile ("nop");
		}
	}
}

volatile uint32_t g_stopwatch_tickCounter = 0;
class stopwatch
{
	uint32_t offset;
	uint32_t time;
	bool running;
public:
	stopwatch()
	{
		offset = g_stopwatch_tickCounter + (TCNT0>>1);
		time = 0;
		running = true;
	}
	void start()
	{
		offset = g_stopwatch_tickCounter + (TCNT0>>1) - time;
		running = true;
	}
	void stop()
	{
		time = g_stopwatch_tickCounter + (TCNT0>>1) - offset;
		running = false;
	}
	void clear()
	{
		offset = g_stopwatch_tickCounter + (TCNT0>>1);
		time = 0;
	}
	uint32_t getTime()
	{
		if(running)
			return g_stopwatch_tickCounter + (TCNT0>>1) - offset;
		return time;
	}
};

ISR(TIMER0_OVF_vect)
{
	g_stopwatch_tickCounter += 128;
}

void wait(const uint32_t &time)
{
	stopwatch s;
	while(time > s.getTime()){}
}

class dynamixel_t {

	queue <uint8_t, DYNAMIXEL_BUS_RX_BUF> m_rx;
	queue <uint8_t, DYNAMIXEL_BUS_TX_BUF> m_tx;

public:

	void data_in(const uint8_t &ch)
	{
		while(!m_rx.push(ch)) {}
	}

	bool data_out(uint8_t &ch)
	{
		if(m_tx.is_empty())
			return false;
		ch = m_tx.pop();
		return true;
	}

//linka

	void transmit()
	{
		PORTE = (PORTE & ~(1<<PE3))|(1<<PE2);
		UCSR0B = (1<<TXEN0);
	}

	void receive()
	{
		PORTE = (PORTE & ~(1<<PE2))|(1<<PE3);
		syncWait(10);
		UCSR0B = (1<<RXCIE0)|(1<<RXEN0);
	}

	void init(uint32_t speed)
	{
		m_tx.overwrite = false;
		m_rx.overwrite = false;
		UCSR0A = (1<<U2X0);
		UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
		UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);
		speed = (((F_CPU/(float(8*speed)))-1)-((F_CPU/(8*speed))-1))<0.5?((F_CPU/(8*speed))-1):((F_CPU/(8*speed)));
		UBRR0H = ((speed&0xFF00)>>8);
		UBRR0L = (speed&0x00FF);
		DDRE |= (1<<PE2)|(1<<PE3);
		receive();
	}

	bool is_send()
	{
		if(!m_tx.is_empty() && ((UCSR0A & (1<<TXC0)) == 0))
			return false;
		UCSR0A |= (1<<TXC0);
		return true;
	}

	void sendChar(uint8_t data)
	{
		while(!m_tx.push(data)) {}
		//pc<<data<<'t';
		UCSR0B |= (1<<UDRIE0);
	}

	void wait()
	{
		while(!is_send()) {}
	}

	bool peek(uint8_t & data)
	{
		if(m_rx.is_empty())
			return false;
		data = m_rx.pop();
		return true;
	}

	void send(uint8_t * data, uint8_t length)
	{
		for(uint8_t i = 0; i < length; ++i)
			sendChar(*(data + i));
	}

	int get(uint8_t * data, uint8_t length, uint16_t max_time)
	{
		uint16_t waiting = 0;
		uint8_t received = length;
		while(length-- != 0)
		{
			while(!peek(*data))
			{
				if(++waiting == max_time)
				{
					--received;
					//pc<<"breaked"<<endl;
					break;
				}
			}
			//pc<<*data<<"rg";
			++data;
			waiting = 0;
		}
		return received;
	}

//protokol
	int calc_checksum(uint8_t * p, uint8_t length)
	{
		uint16_t checksum = *p;
		for(++p; --length != 0; ++p)
			checksum += *p;
		return ((~checksum) & 255);
	}
#if 0
	int com(uint8_t ID, uint8_t instruction, uint8_t *data, uint16_t Tlength, uint8_t Rlength, uint32_t retWait, uint16_t max_time = 500)
	{
		transmit();
		uint8_t buffer [(Tlength>Rlength?Tlength:Rlength) + 6];
		buffer[0] = 255;
		buffer[1] = 255;
		buffer[2] = ID;
		buffer[3] = Tlength + 2;
		buffer[4] = instruction;
		for(uint16_t i = 0; i < Tlength; ++i)
		{
			buffer[i + 5] = *(data + i);
		}
		buffer[Tlength + 5] = calc_checksum(&buffer[2], Tlength + 3);
		//pc.sendNumbers(&buffer[0], Tlength + 6);
		send(&buffer[0], Tlength + 6);
		wait();
		receive();
		if(Rlength != 0)
		{
			#ifdef wait
			::wait(retWait<2?2:retWait);
			#else
			syncWait(retWait<2?2:retWait);
			#endif
			uint8_t received = get(&buffer[0], Rlength + 5, max_time);
			pc<<received<<endl;
			for(uint8_t i = 0; i < Rlength + 5; ++i)
			{
				pc.sendNumber(buffer[i]);
				pc.send("; ");
			}
			pc.send("\n\r");
			if((received > 5)&&(buffer[0] == 255)&&(buffer[1] == 255)&&(buffer[2] == ID)&&(calc_checksum(&buffer[2], buffer[3] + 1) == (buffer[3] + 5)))
			{
				for(uint8_t i = 0; i != (buffer[3] + 1); ++i)
					*(data + i) = buffer[i + 4];
				return (Rlength - (buffer[3] + 1));
			}
			return Rlength;
		}
		return 0;
	}
#endif
#if 1
	int com(uint8_t ID, uint8_t instruction, uint8_t *data, uint16_t Tlength, uint8_t Rlength, uint32_t retWait, uint16_t max_time = 500)
	{
		uint8_t checksum = ID + Tlength + 2 + instruction;
		for(uint8_t i = 0; i < Tlength; ++i)
			checksum += *(data+i);
		checksum = ~checksum;
		uint8_t returned = 0;
		transmit();
		sendChar(0xFF);
		sendChar(0xFF);
		sendChar(ID);
		sendChar(Tlength + 2);
		sendChar(instruction);
		send(data, Tlength);
		sendChar(checksum);
		wait();
		receive();
		while(!peek(returned) || returned != 255) {}
		//pc<<returned<<'r';
		while(!peek(returned) || returned != 255) {}
		//pc<<returned<<'r';
		while(!peek(returned) || returned !=  ID) {}
		//pc<<returned<<'r';
		while(!peek(returned)){}
		//pc<<returned<<"re";
		uint8_t a = get(data, (returned - 1), max_time);
		//pc<<endl<<"a: "<<a<<Rlength<<endl;
		if(a == Rlength)
		{
			checksum = ID + returned;
			for(uint8_t i = 0; i != returned - 1; ++i)
				checksum += *(data+i);
			checksum = ~checksum;
			while(!peek(returned)){}
			//pc<<returned<<'r'<<endl<<"checksum: "<<checksum<<endl;
			if(checksum == returned)
			{
				//pc<<"com return 0"<<endl;
				return 0;
			}
		}
		//pc<<endl<<"spatna delka: "<<a<<Rlength<<endl;
		return -1;
	}
#endif
	int readByte(uint8_t ID, uint8_t address, uint8_t &data)
	{
		uint8_t repository[2] = {address, 1};
		repository[0] = com(ID, INST_READ, &repository[0], 2, DYNAMIXEL_RETURN!=0?2:0, DYNAMIXEL_RESPOND_TIME)==0?repository[0]:(repository[0]|(1<<7));
		data = repository[1];
		return repository[0];
	}
	int readWord(uint8_t ID, uint8_t address, uint16_t &data)
	{
		uint8_t repository[3] = {address, 2};
		repository[0] = com(ID, INST_READ, &repository[0], 2, DYNAMIXEL_RETURN!=0?3:0, DYNAMIXEL_RESPOND_TIME)==0?repository[0]:(repository[0]|(1<<7));
		data = repository[1] | (repository[2] << 8);
		return repository[0];
	}
	int writeByte(uint8_t ID, uint8_t address, uint8_t data)
	{
		uint8_t repository[2] = {address, data};
		return com(ID, INST_WRITE, &repository[0], 2, DYNAMIXEL_RETURN==2?1:0, DYNAMIXEL_RESPOND_TIME)==0?repository[0]:(repository[0]|(1<<7));
	}
	int writeWord(uint8_t ID, uint8_t address, uint16_t data)
	{
		uint8_t repository[3] = {address, (data & 0x00FF), (data & 0xFF00) >> 8};
		return com(ID, INST_WRITE, &repository[0], 3, DYNAMIXEL_RETURN==2?1:0, DYNAMIXEL_RESPOND_TIME)==0?repository[0]:(repository[0]|(1<<7));
	}
	int writeDWord(uint8_t ID, uint8_t address, uint16_t data0, uint16_t data1)
	{
		uint8_t repository[5] = {address, (data0 & 0x00FF), (data0 & 0xFF00) >> 8, (data1 & 0x00FF), (data1 & 0xFF00) >> 8};
		return com(ID, INST_WRITE, &repository[0], 5, DYNAMIXEL_RETURN==2?1:0, DYNAMIXEL_RESPOND_TIME)==0?repository[0]:(repository[0]|(1<<7));
	}
	int regWriteByte(uint8_t ID, uint8_t address, uint8_t data)
	{
		uint8_t repository[2] = {address, data};
		return com(ID, INST_REG_WRITE, &repository[0], 2, DYNAMIXEL_RETURN==2?1:0, DYNAMIXEL_RESPOND_TIME)==0?repository[0]:(repository[0]|(1<<7));
	}
	int regWriteWord(uint8_t ID, uint8_t address, uint16_t data)
	{
		uint8_t repository[3] = {address, (data & 0x00FF), (data & 0xFF00) >> 8};
		return com(ID, INST_REG_WRITE, &repository[0], 3, DYNAMIXEL_RETURN==2?1:0, DYNAMIXEL_RESPOND_TIME)==0?repository[0]:(repository[0]|(1<<7));
	}
	int regWriteDWord(uint8_t ID, uint8_t address, uint16_t data0, uint16_t data1)
	{
		uint8_t repository[5] = {address, (data0 & 0x00FF), (data0 & 0xFF00) >> 8, (data1 & 0x00FF), (data1 & 0xFF00) >> 8};
		return com(ID, INST_REG_WRITE, &repository[0], 5, DYNAMIXEL_RETURN==2?1:0, DYNAMIXEL_RESPOND_TIME)==0?repository[0]:(repository[0]|(1<<7));
	}
	void syncWriteByte(uint8_t ID[], uint8_t address, uint8_t data[], uint8_t IDs)
	{
		uint8_t repository[IDs * 2 + 2];
		repository[0] = address;
		repository[1] = 1;
		for(uint8_t i = 0; i < IDs; ++i)
		{
			repository[2 * (i + 1)] = ID[i];
			repository[2 * (i + 1) + 1] = data[i];
		}
		com(BROADCASTING_ID, INST_SYNC_WRITE, &repository[0], IDs * 2 + 2, 0, DYNAMIXEL_RESPOND_TIME);
	}
	void syncWriteWord(uint8_t ID[], uint8_t address, uint16_t data[], uint8_t IDs)
	{
		uint8_t repository[IDs * 3 + 2];
		repository[0] = address;
		repository[1] = 2;
		for(uint8_t i = 0; i < IDs; ++i)
		{
			repository[3 * (i + 1)] = ID[i];
			repository[3 * (i + 1) + 1] = (data[i] & 0x00FF);
			repository[3 * (i + 1) + 2] = (data[i] & 0xFF00) >> 8;
		}
		com(BROADCASTING_ID, INST_SYNC_WRITE, &repository[0], IDs * 3 + 2, 0, DYNAMIXEL_RESPOND_TIME);
	}
	void syncWriteDWord(uint8_t ID[], uint8_t address, uint16_t data0[], uint16_t data1[], uint8_t IDs)
	{
		uint8_t repository[IDs * 5 + 2];
		repository[0] = address;
		repository[1] = 2;
		for(uint8_t i = 0; i < IDs; ++i)
		{
			repository[5 * (i + 1)] = ID[i];
			repository[5 * (i + 1) + 1] = (data0[i] & 0x00FF);
			repository[5 * (i + 1) + 2] = (data0[i] & 0xFF00) >> 8;
			repository[5 * (i + 1) + 3] = (data1[i] & 0x00FF);
			repository[5 * (i + 1) + 4] = (data1[i] & 0xFF00) >> 8;
		}
		com(BROADCASTING_ID, INST_SYNC_WRITE, &repository[0], IDs * 5 + 2, 0, DYNAMIXEL_RESPOND_TIME);
	}
	void syncRegWriteByte(uint8_t ID[], uint8_t address, uint8_t data[], uint8_t IDs)
	{
		uint8_t repository[IDs * 2 + 2];
		repository[0] = address;
		repository[1] = 1;
		for(uint8_t i = 0; i < IDs; ++i)
		{
			repository[2 * (i + 1)] = ID[i];
			repository[2 * (i + 1) + 1] = data[i];
		}
		com(BROADCASTING_ID, INST_SYNC_REG_WRITE, &repository[0], IDs * 2 + 2, 0, DYNAMIXEL_RESPOND_TIME);
	}
	void syncRegWriteWord(uint8_t ID[], uint8_t address, uint16_t data[], uint8_t IDs)
	{
		uint8_t repository[IDs * 3 + 2];
		repository[0] = address;
		repository[1] = 2;
		for(uint8_t i = 0; i < IDs; ++i)
		{
			repository[3 * (i + 1)] = ID[i];
			repository[3 * (i + 1) + 1] = (data[i] & 0x00FF);
			repository[3 * (i + 1) + 2] = (data[i] & 0xFF00) >> 8;
		}
		com(BROADCASTING_ID, INST_SYNC_REG_WRITE, &repository[0], IDs * 3 + 2, 0, DYNAMIXEL_RESPOND_TIME);
	}
	void syncRegWriteDWord(uint8_t ID[], uint8_t address, uint16_t data0[], uint16_t data1[], uint8_t IDs)
	{
		uint8_t repository[IDs * 5 + 2];
		repository[0] = address;
		repository[1] = 2;
		for(uint8_t i = 0; i < IDs; ++i)
		{
			repository[5 * (i + 1)] = ID[i];
			repository[5 * (i + 1) + 1] = (data0[i] & 0x00FF);
			repository[5 * (i + 1) + 2] = (data0[i] & 0xFF00) >> 8;
			repository[5 * (i + 1) + 3] = (data1[i] & 0x00FF);
			repository[5 * (i + 1) + 4] = (data1[i] & 0xFF00) >> 8;
		}
		com(BROADCASTING_ID, INST_SYNC_REG_WRITE, &repository[0], IDs * 5 + 2, 0, DYNAMIXEL_RESPOND_TIME);
	}
	int action(uint8_t ID)
	{
		uint8_t repository[1];
		return com(ID, INST_ACTION, &repository[0], 3, DYNAMIXEL_RETURN==2?1:0, DYNAMIXEL_RESPOND_TIME)==0?repository[0]:(repository[0]|(1<<7));
	}
	int ping(uint8_t ID)
	{
		uint8_t repository[1];
		return com(ID, INST_PING, &repository[0], 3, DYNAMIXEL_RETURN==2?1:0, DYNAMIXEL_RESPOND_TIME)==0?repository[0]:(repository[0]|(1<<7));
	}

}; dynamixel_t bus;

ISR(USART0_RX_vect)
{
	char ch = UDR0;
	if((UCSR0A & (1<<FE0)) == (1<<FE0))
		return;
	bus.data_in(ch);
}
ISR(USART0_UDRE_vect)
{
	uint8_t data;
	if(bus.data_out(data))
	{
		//PORTE &= ~(1<<PE3);
		//PORTE |= (1<<PE2);
		UDR0 = data;
	}
	else
	{
		//PORTE &= ~(1<<PE2);
		//PORTE |= (1<<PE3);
		UCSR0B &= ~(1<<UDRIE0);
	}
}

//********************** simplifying functions for AX-12**********************************
/*int setPositionLimits(uint8_t ID, uint16_t downLimit, uint16_t upLimit)
{
	return bus.write2w(ID, P_CW_ANGLE_LIMIT_L, downLimit, upLimit);
}
int setMotorPosition(uint8_t ID, uint16_t position)
{
	return bus.write16b(ID, P_GOAL_POSITION_L, position);
}
int regSetMotorPosition(uint8_t ID, uint16_t position)
{
	return bus.regWrite16b(ID, P_GOAL_POSITION_L, position);
}
int setMotorSpeed(uint8_t ID, uint16_t speed)
{
	return bus.write16b(ID, P_GOAL_SPEED_L, speed);
}
int setMotorLed(uint8_t ID)
{
	return bus.write8b(ID, P_LED, 1);
}
int clearMotorLed(uint8_t ID)
{
	return bus.write8b(ID, P_LED, 0);
}
int toggleMotorLed(uint8_t ID)
{
	uint8_t data = 0;
	uint8_t error = bus.read8b(ID, P_LED, data);
	if(error == 0)
	{
		if(data == 0)
		{
			return bus.write8b(ID, P_LED, 1);
		}
		else
		{
			return bus.write8b(ID, P_LED, 0);
		}
	}
	return error;
}
int setTorqueLimit(uint8_t ID, uint16_t torque)
{
	return bus.write16b(ID, P_TORQUE_LIMIT_L, torque);
}
int setTorque(uint8_t ID) //yapnut9 motoru
{
	return bus.write8b(ID, P_TORQUE_ENABLE, 1);
}
int clearTorque(uint8_t ID)
{
	return bus.write8b(ID, P_TORQUE_ENABLE, 0);
}
int getMotorLoad(uint8_t ID)
{
	uint16_t load;
	uint8_t error = bus.read16b(ID, P_PRESENT_LOAD_L, load);
	if(error == 0)
		return (((load & (1<<10)) == 0)?(-(load&((1<<10)-1))):(load&((1<<10)-1)));
	return (1<<14)+error;
}
int getMotorPosition(uint8_t ID)
{
	uint16_t position;
	uint8_t error = bus.read16b(ID, P_PRESENT_POSITION_L, position);
	if(error == 0)
		return position;
	return (1<<14)+error;
}*/
class motor_t
{
public:
	uint8_t currentID;
	queue <uint8_t, AX_ERROR_BUF> m_error;

	motor_t()
	{
		m_error.overwrite = true;
	}

	motor_t &operator [] (const uint8_t &id)
	{
		currentID = id;
		return *this;
	}
	motor_t &operator << (const uint16_t &position)
	{
		m_error.push(bus.writeWord(currentID, P_GOAL_POSITION_L, position));
		return *this;
	}
	motor_t &operator << (const int16_t &position)
	{
		m_error.push(bus.writeWord(currentID, P_GOAL_POSITION_L, position));
		return *this;
	}
	motor_t &operator >> (uint16_t &position)
	{
		m_error.push(bus.readWord(currentID, P_PRESENT_POSITION_L, position));
		return *this;
	}
	motor_t &operator >> (int16_t &position)
	{
		uint16_t temp;
		m_error.push(bus.readWord(currentID, P_PRESENT_POSITION_L, temp));
		position = temp;
		return *this;
	}
	uint8_t error()
	{
		return m_error.pop();
	}
	motor_t & setTorque()
	{
		m_error.push(bus.writeByte(currentID, P_TORQUE_ENABLE, 1));
		return *this;
	}
	motor_t & clearTorque()
	{
		m_error.push(bus.writeByte(currentID, P_TORQUE_ENABLE, 0));
		return *this;
	}
	motor_t & torqueLimit(const uint16_t &limit = 1023)
	{
		m_error.push(bus.writeByte(currentID, P_TORQUE_LIMIT_L, limit));
		return *this;
	}
	motor_t & torqueLimit(const int16_t &limit = 1023)
	{
		m_error.push(bus.writeWord(currentID, P_TORQUE_LIMIT_L, ((uint16_t)(limit))));
		return *this;
	}
	motor_t & CWlimit(const uint16_t &limit = 1023)
	{
		m_error.push(bus.writeWord(currentID, P_CW_ANGLE_LIMIT_L, limit));
		return *this;
	}
	motor_t & CWlimit(const int16_t &limit = 1023)
	{
		m_error.push(bus.writeWord(currentID, P_CW_ANGLE_LIMIT_L, ((uint16_t)(limit))));
		return *this;
	}
	motor_t & CCWlimit(const uint16_t &limit = 1023)
	{
		m_error.push(bus.writeWord(currentID, P_CCW_ANGLE_LIMIT_L, limit));
		return *this;
	}
	motor_t & CCWlimit(const int16_t &limit = 1023)
	{
		m_error.push(bus.writeWord(currentID, P_CCW_ANGLE_LIMIT_L, ((uint16_t)(limit))));
		return *this;
	}
	motor_t & speed(const uint16_t &speed = 256)
	{
		m_error.push(bus.writeWord(currentID, P_GOAL_SPEED_L, speed));
		return *this;
	}
	motor_t & speed(const int16_t &speed = 256)
	{
		m_error.push(bus.writeWord(currentID, P_GOAL_SPEED_L, ((uint16_t)(speed))));
		return *this;
	}
	motor_t & setLed()
	{
		m_error.push(bus.writeWord(currentID, P_LED, 1));
		return *this;
	}
	motor_t & clearLed()
	{
		m_error.push(bus.writeWord(currentID, P_LED, 0));
		return *this;
	}
	motor_t & toggleLed()
	{
		uint8_t temp;
		uint8_t error = bus.readByte(currentID, P_LED, temp);
		if(error == 0)
		{
			if(temp == 0)
				setLed();
			else
				clearLed();
		}
		m_error.push(error);
		return *this;
	}
	motor_t & position(const uint16_t &position)
	{
		m_error.push(bus.writeWord(currentID, P_GOAL_POSITION_L, position));
		return *this;
	}
	motor_t & position(const int16_t &position)
	{
		m_error.push(bus.writeWord(currentID, P_GOAL_POSITION_L, ((uint16_t)(position))));
		return *this;
	}
	motor_t & position(const uint16_t &position, const uint16_t &speed)
	{
		m_error.push(bus.writeDWord(currentID, P_GOAL_POSITION_L, position, speed));
		return *this;
	}
	motor_t & position(const int16_t &position, const int16_t &speed)
	{
		m_error.push(bus.writeDWord(currentID, P_GOAL_POSITION_L, ((uint16_t)(position)), ((uint16_t)(speed))));
		return *this;
	}
	uint16_t position()
	{
		uint16_t pos;
		m_error.push(bus.readWord(currentID, P_PRESENT_POSITION_L, pos));
		return pos;
	}
}; motor_t motor;

class sensor_t
{
	public:
	uint8_t currentID;
	queue <uint8_t, AX_ERROR_BUF> m_error;

	sensor_t()
	{
		m_error.overwrite = true;
		currentID = 100;
	}

	sensor_t &operator [] (const uint8_t &id)
	{
		currentID = id;
		return *this;
	}
	uint8_t error()
	{
		return m_error.pop();
	}
	uint8_t leftDistance()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_LEFT_IR_SENSOR_DATA, d));
		return d;
	}
	uint8_t centerDistance()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_CENTER_IR_SENSOR_DATA, d));
		return d;
	}
	uint8_t rightDistance()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_RIGHT_IR_SENSOR_DATA, d));
		return d;
	}
	uint8_t leftLuminosity()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_LEFT_LUMINOSITY, d));
		return d;
	}
	uint8_t centerLuminosity()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_CENTER_LUMINOSITY, d));
		return d;
	}
	uint8_t rightLuminosity()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_RIGHT_LUMINOSITY, d));
		return d;
	}
	uint8_t voltage()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_PRESENT_VOLTAGE, d));
		return d;
	}
	uint8_t temperature()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_PRESENT_TEMPERATURE, d));
		return d;
	}
	void bstacleDetectionLine(const uint8_t &d)
	{
		m_error.push(bus.writeByte(currentID, P_OBSTACLE_DETECTED_COMPARE, d));
	}
	uint8_t obstacleDetectionLine()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_OBSTACLE_DETECTED_COMPARE, d));
		return d;
	}
	void lightDetection(const uint8_t &d)
	{
		m_error.push(bus.writeByte(currentID, P_LIGHT_DETECTED_COMPARE, d));
	}
	uint8_t lightDetection()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_LIGHT_DETECTED_COMPARE, d));
		return d;
	}
	void eepromObstacleDetectionLine(const uint8_t &d)
	{
		m_error.push(bus.writeByte(currentID, P_OBSTACLE_DETECTED_COMPARE_VALUE, d));
	}
	uint8_t eepromObstacleDetectionLine()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_OBSTACLE_DETECTED_COMPARE_VALUE, d));
		return d;
	}
	void eepromLightDetection(const uint8_t &d)
	{
		m_error.push(bus.writeByte(currentID, P_LIGHT_DETECTED_COMPARE_VALUE, d));
	}
	uint8_t eepromLightDetection()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_LIGHT_DETECTED_COMPARE_VALUE, d));
		return d;
	}
	uint8_t isObstacle()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_OBSTACLE_DETECTION_FLAG, d));
		return d;
	}
	bool isLeftObstacle()
	{
		return ((isObstacle() & 1) != 0)?true:false;
	}
	bool isCenterObstacle()
	{
		return ((isObstacle() & 2) != 0)?true:false;
	}
	bool isRightObstacle()
	{
		return ((isObstacle() & 4) != 0)?true:false;
	}
	uint8_t isLight()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_LUMINOSITY_DETECTION_FLAG, d));
		return d;
	}
	bool isLeftLight()
	{
		return ((isLight() & 1) != 0)?true:false;
	}
	bool isCenterLight()
	{
		return ((isLight() & 2) != 0)?true:false;
	}
	bool isRightLight()
	{
		return ((isLight() & 4) != 0)?true:false;
	}
	uint8_t soundData()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_SOUND_DATA, d));
		return d;
	}
	uint8_t noise()
	{
		return abs(128-soundData());
	}
	uint8_t maxSoundData()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_SOUND_DATA_MAX_HOLD, d));
		return d;
	}
	void ClearMaxSoundData()
	{
		m_error.push(bus.writeByte(currentID, P_SOUND_DATA_MAX_HOLD, 0));
	}
	uint8_t soundDetectedCount()
	{
		uint8_t d;
		m_error.push(bus.readByte(currentID, P_SOUND_DETECTED_COUNT, d));
		return d;
	}
	void clearSoundDetectedCount()
	{
		m_error.push(bus.writeByte(currentID, P_SOUND_DETECTED_COUNT	, 0));
	}
	uint16_t soundDetectedTime()
	{
		uint16_t d;
		m_error.push(bus.readWord(currentID, P_SOUND_DETECTED_TIME_L, d));
		return d;
	}
	void clearSoundDetectedTimes(uint8_t IDs[], uint8_t &length)
	{
		uint16_t temp[length];
		for(uint8_t i = 0; i != length; ++i)
			temp[i] = 0;
		bus.syncWriteWord(IDs, P_SOUND_DETECTED_TIME_L, temp, length);
	}
	void buzzerTone(const uint8_t &tone)
	{
		m_error.push(bus.writeByte(currentID, P_BUZZER_INDEX, tone));
	}
	void buzzerTime(const uint8_t &time)
	{
		m_error.push(bus.writeByte(currentID, P_BUZZER_TIME, time));
	}
	bool peek(uint16_t &data)
	{
		uint8_t isReceived;
		m_error.push(bus.readByte(currentID, P_IR_REMOCON_ARRIVED, isReceived));
		if(isReceived == 0)
			return false;
		m_error.push(bus.readWord(currentID, P_IR_REMOCON_RX_DATA_0, data));
		return true;
	}
	uint16_t get()
	{
		uint16_t temp;
		while(!peek(temp)){}
		return temp;
	}
	void send(const uint16_t &data)
	{
		m_error.push(bus.writeWord(currentID, P_IR_REMOCON_TX_DATA_0, data));
	}
}; sensor_t sensor;

class button_t {

public:

	bool isUp()
	{
		return((PINE & (1<<PE4)) == 0)?true:false;
	}
	bool isDown()
	{
		return((PINE & (1<<PE5)) == 0)?true:false;
	}
	bool isLeft()
	{
		return((PINE & (1<<PE6)) == 0)?true:false;
	}
	bool isRight()
	{
		return((PINE & (1<<PE7)) == 0)?true:false;
	}
	bool isStart()
	{
		return((PIND & (1<<PD0)) == 0)?true:false;
	}
private:
	void click(bool (button_t::* is_pressed)())
	{
		bool dont_pass = true;
		while(dont_pass)
		{
			while(!((this->*is_pressed)())) {}
			stopwatch enough;
			enough.start();

			while ((this->*is_pressed)())
			{
			}

			if(enough.getTime() < 50000)
				continue;
			enough.clear();
			while(!((this->*is_pressed)()))
			{
				if(enough.getTime() > 50000)
				{
					dont_pass = false;
					break;
				}
			}
		}	
	}
public:
	void start()
	{
		click(&button_t::isStart);
	}
	void up()
	{
		click(&button_t::isUp);
	}
	void down()
	{
		click(&button_t::isDown);
	}
	void left()
	{
		click(&button_t::isLeft);
	}
	void right()
	{
		click(&button_t::isRight);
	}

}; button_t buttons;

class led_t {

public:

	void txd(uint8_t stav)
	{
		switch(stav)
		{
			case 0:
				PORTC |= (1<<PC1);
				break;

			case 1:
				PORTC &= ~(1<<PC1);
				break;

			case 2:
				if((PINC & (1<<PC1)) == 0)
					PORTC |= (1<<PC1);
				else
					PORTC &= ~(1<<PC1);
				break;
		}
	}
	void rxd(uint8_t stav)
	{
		switch(stav)
		{
			case 0:
				PORTC |= (1<<PC2);
				break;

			case 1:
				PORTC &= ~(1<<PC2);
				break;

			case 2:
				if((PINC & (1<<PC2)) == 0)
					PORTC |= (1<<PC2);
				else
					PORTC &= ~(1<<PC2);
				break;
		}
	}
	void aux(uint8_t stav)
	{
		switch(stav)
		{
			case 0:
				PORTC |= (1<<PC3);
				break;

			case 1:
				PORTC &= ~(1<<PC3);
				break;

			case 2:
				if((PINC & (1<<PC3)) == 0)
					PORTC |= (1<<PC3);
				else
					PORTC &= ~(1<<PC3);
				break;
		}
	}
	void manage(uint8_t stav)
	{
		switch(stav)
		{
			case 0:
				PORTC |= (1<<PC4);
				break;

			case 1:
				PORTC &= ~(1<<PC4);
				break;

			case 2:
				if((PINC & (1<<PC4)) == 0)
					PORTC |= (1<<PC4);
				else
					PORTC &= ~(1<<PC4);
				break;
		}
	}
	void program(uint8_t stav)
	{
		switch(stav)
		{
			case 0:
				PORTC |= (1<<PC5);
				break;

			case 1:
				PORTC &= ~(1<<PC5);
				break;

			case 2:
				if((PINC & (1<<PC5)) == 0)
					PORTC |= (1<<PC5);
				else
					PORTC &= ~(1<<PC5);
				break;
		}
	}
	void play(uint8_t stav)
	{
		switch(stav)
		{
			case 0:
				PORTC |= (1<<PC6);
				break;

			case 1:
				PORTC &= ~(1<<PC6);
				break;

			case 2:
				if((PINC & (1<<PC6)) == 0)
					PORTC |= (1<<PC6);
				else
					PORTC &= ~(1<<PC6);
				break;
		}
	}
	void power(uint8_t stav)
	{
		switch(stav)
		{
			case 0:
				PORTC |= (1<<PC0);
				break;

			case 1:
				PORTC &= ~(1<<PC0);
				break;

			case 2:
				if((PINC & (1<<PC0)) == 0)
					PORTC |= (1<<PC0);
				else
					PORTC &= ~(1<<PC0);
				break;
		}
	}
	void dispalyNumber(uint8_t n)
	{
		if((n & 1) == 1)
			play(1);
		else
			play(0);
		if((n & 2) == 2)
			program(1);
		else
			program(0);
		if((n & 4) == 4)
			manage(1);
		else
			manage(0);
		if((n & 8) == 8)
			aux(1);
		else
			aux(0);
		if((n & 16) == 16)
			rxd(1);
		else
			rxd(0);
		if((n & 32) == 32)
			txd(1);
		else
			txd(0);
	}

}; led_t led;

class eeprom_t {
public:
	uint8_t readByte(const uint16_t &address)
	{
		if(address > 4095)
			return 0xFF;
		EEAR = address;
		EECR |= (1<<EERE);
		return EEDR;
	}
	void writeByte(const uint16_t &address, const uint8_t &data)
	{
		if(address < 4096)
		{
			cli();
			EEAR = address;
			EEDR = data;
			EECR = (1<<EEMWE);
			EECR |= (1<<EEWE);
			while((EECR & (1<<EEWE)) == (1<<EEWE));
			{
			}
			sei();
		}
	}
	void writeWord(const uint16_t &address, const uint16_t &data)
	{
		writeByte(address, (data & 0xFF00) >> 8);
		writeByte((address + 1), (data & 0x00FF));
	}
	uint16_t readWord(const uint16_t &address)
	{
		return (readByte(address)<<8)|readByte(address + 1);
	}
	/*void write_float(uint16_t address, float data)
	{
		int16_t temp = data;
		write_word(address, temp);
		temp = ((data - temp)*EEPROM_FIXPOINT);
		write_word(address + 2, temp);
	}
	float read_float(uint16_t address)
	{
		return (read_word(address + 2)/EEPROM_FIXPOINT) + read_word(address);
	}*/
	void writeDWord(const uint16_t &address, const uint32_t &data)
	{
		writeWord(address, (data & 0xFFFF0000) >> 16);
		writeWord((address + 2), (data & 0x0000FFFF));
	}
	int32_t readDWord(const uint16_t &address)
	{
		return (((int32_t)(readDWord(address)))<<16)|readDWord(address + 2);
	}
}; eeprom_t eeprom;


void run();

int main()
{
	//LEDs initialization
	DDRC = (1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5)|(1<<PC6);
	PORTC = (1<<PC1)|(1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5)|(1<<PC6);
	//buttons initialization
	DDRE = (1<<PE2)|(1<<PE3);
	PORTE = (1<<PE7)|(1<<PE6)|(1<<PE5)|(1<<PE4)|(1<<PE0);
	DDRD = (1<<PD5);
	PORTD = (1<<PD0)|(1<<PD5);
	// Initialize the RS232 line to PC
	pc.init(DYNAMIXEL_PC_BPS);
	//Dynamixel bus initialization
	bus.init(DYNAMIXEL_BUS_BPS);
	//stopwatch initialization
	TIMSK = (1<<TOIE0);
	TCCR0 = (1<<CS01);
	//starting program
	sei();
	syncWait(1000);
	pc.send("\n\n\r  ok \n\r");
	pc.wait();
	run();//user program
	pc.send("\n\rEnd of program!\n\n\r");
	pc.wait();
	for(;;){}
}
