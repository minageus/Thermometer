////////////////////////////////////////////////////////////////////////////////
// Module ��� ������������ QN1610GT ��� ������������ ��� �����������
// ��������� ������������.
// ���������� ���� �� �� USART0.
////////////////////////////////////////////////////////////////////////////////

// �� header ��� ��������� module (������������ �� ������ header BaseHeader.h).
#include "Serial.h"

// �� �������� headers ��� ������������.


////////////////////////////////////////////////////////////////////////////////
// Global �������� ��� �� �������� �����������.
// ������ �� ���������� ��� �������� ���6 ���� ������ ��������, ��������������
// uint8_t ��� ��������.
////////////////////////////////////////////////////////////////////////////////

// �� �������� buffers ��� ������ ��� ����� ����������.
uint8_t g_taSerialInputBuffer[SERIAL_INPUT_BUFFER_SIZE];
uint8_t g_taSerialOutputBuffer[SERIAL_OUTPUT_BUFFER_SIZE];

// ���� ���������� �������� �������� ���� ���� buffer �������.
// ����������� �� ����� ���� ��� ��� SERIAL_INPUT_BUFFER_SIZE.
uint8_t g_tSerialInputBufferDataSize = 0;
// Index ���� ���� buffer ������� ��� ������� ��� ����� �� ����� byte
// ��� ����.
uint8_t g_tSerialInputBufferFirstDataByteIndex = 0;

// ���� ���������� �������� �������� ���� ���� buffer ������.
// ����������� �� ����� ���� ��� ��� SERIAL_OUTPUT_BUFFER_SIZE.
uint8_t g_tSerialOutputBufferDataSize = 0;
// Index ���� ���� buffer ������ ��� ������� ��� ����� �� ����� byte
// ��� ����� ��� �� ����� ��� ��������.
uint8_t g_tSerialOutputBufferFirstDataByteIndex = 0;

// �� ��������� ��� WriteData �� ������� ��� byte ��� ��� ������ ��������
// ���� buffer, � WriteData ������� �� byte ��' ������� ��� UDR. �� ���������
// ���� ������ ��� WriteData, ���� ��� �� ����� ��� ���� �� ������
// ���������� �������� ��� �� ���������� ���� UDR. ������� ������ �� ���������
// m_bSerialSendingByte ��� �� ������� ���������� �� ������ ������ �������
// ��������.
bool g_bSerialSendingByte = false;


////////////////////////////////////////////////////////////////////////////////
// ����������� ��� module.
////////////////////////////////////////////////////////////////////////////////


// ��������� �� ��� ����� ������������� �� �������� �����������.
// �������� ��� �� USART ��� ��� ���������� ���.
// � ���������� p_rUbrr �������� ��� �������� ��� �� �������� ���� ����������
// UBRR ��� �� ������� �� baud rate.
void SerialInitialize(uint16_t p_rUbrr)
{
	uint8_t l_tStatusRegister;
	
	l_tStatusRegister = SREG;
	cpu_irq_disable();
	// �� � �������� ������� ����� ��� ��������������, �� ������ �� �����������
	// �� ��������� (���� ���� �� datasheet ��� AVR) ���� ��� ��������� baud rate.
	// ������ ����� ��� �� ��������� �� baud rate ��������� ����� ������� �� ��������,
	// ���� ��� ��������, ���� ����� �� ����������� �� ��������� � ����� �������.
	if (UCSR0B & _BV(TXEN0))
	{
		cpu_irq_enable();
		while (!SerialIsTransmitionComplete());
		cpu_irq_disable();
	}
	// ��������������� ������� �� ��������, �������� ��� ����� ��� ������� ��� reset.
	// �������� ����� ��� UCSR0B ��� ������� �� ��������.
	UCSR0B = 0x00;
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	// ������������� ��� ���������� ���.
	g_tSerialInputBufferDataSize = 0;
	g_tSerialInputBufferFirstDataByteIndex = 0;
	g_tSerialOutputBufferDataSize = 0;
	g_tSerialOutputBufferFirstDataByteIndex = 0;
	g_bSerialSendingByte = false;

	// ������� �� baud rate.
	// ��������� ��' ������� ���� UBRR0 (16-bit �����������), � compiler ����� �����
	// �� �������� ����� � UBRR0H ��� ���� � UBRR0L.
	UBRR0 = p_rUbrr;
	// � �������� ������������ ��� ����������� ��� USART0 ���� ����� 8 bits, 1 stop bit,
	// no parity, ��� ����� �� �������������� ��� �������. ���� ��� ������������ ������
	// ��������� ������������.
    // ������������� ��� ������� ��� �� ���� ���� �� �� interrupt ����.
	UCSR0B = _BV(RXCIE0) | _BV(TXCIE0) | _BV(RXEN0) | _BV(TXEN0);
	SREG = l_tStatusRegister;
}


// ��������� ��� �������� ��� byte ��� ��� ������� �������� buffer �������,
// ��� ����� ���� p_tpData ��� ���������� true. �� ��� ������� byte ���������,
// ���������� false.
bool SerialReadData(uint8_t *p_tpData)
{
	uint8_t l_tStatusRegister;
	
	l_tStatusRegister = SREG;
	cpu_irq_disable();
	if (g_tSerialInputBufferDataSize == 0)
	{
		SREG = l_tStatusRegister;
		return false;
	}
	*p_tpData = g_taSerialInputBuffer[g_tSerialInputBufferFirstDataByteIndex];
	g_tSerialInputBufferFirstDataByteIndex++;
	if (g_tSerialInputBufferFirstDataByteIndex >= SERIAL_INPUT_BUFFER_SIZE)
	{
		g_tSerialInputBufferFirstDataByteIndex = 0;
	}
	g_tSerialInputBufferDataSize--;
	SREG = l_tStatusRegister;
	return true;
}


// ��������� ��� ����� �������� ���� ������� �������� buffer ������.
// �� � buffer ������ ��� ���� ������ ����, ��������� ����� �� ����.
void SerialWriteData(uint8_t *p_tpData, uint8_t p_tDataLength)
{
	uint8_t l_tStatusRegister;
	uint8_t *l_tpTemp;
	
	if (p_tDataLength == 0)
	{
		return;
	}
	// �� �� �������� ��� ������� �� ��������� ����� ����������� ��� �� �������
	// ���� ��� buffer, ����.
	if (p_tDataLength > SERIAL_OUTPUT_BUFFER_SIZE)
	{
		return;
	}
	// ����� �������� �� ���� ������ ���� � buffer ������. ������ ��� ��� ���������
	// ���� ��� �������� ��� �� ����������� interrupt, �������� �� ��� ���������
	// �������� ����� ���� ����� ��� byte.
	l_tStatusRegister = SREG;
	cpu_irq_disable();
	while (p_tDataLength > SERIAL_OUTPUT_BUFFER_SIZE - g_tSerialOutputBufferDataSize)
	{
		cpu_irq_enable();
		wdt_reset();
		nop();
		nop();
		nop();
		nop();
		cpu_irq_disable();
	}
	// ��� ������ ���� output buffer ������ ���� ��� �� ��������� �� �������� ���.
	// ��� ��� ����� ������ ��������������� �� interrupts, ��� ��� ������� ���������
	// �� ������ � ��������� interrupt ��� ������� ��� ��������, ����� �� �����
	// false � m_bSerialSendingByte. �� ���, ���� ��� ������� ���������� ��� ���������,
	// ����� ������ �� ������� ��� ����� ��������� �� ������, ��� �� ���������
	// ���� ������ ���� output buffer, ������ ���� ������� ����� ���� buffer �����
	// �� ��������� ��� UDR.
	if (!g_bSerialSendingByte)
	{
		// ���� �������� ���� UDR0, ����� ����������� �� ������ bit (�� ��������� ���
		// ������ ����� 9-bits).
		UCSR0B &= ~_BV(TXB80);
		UDR0 = *p_tpData;
		p_tpData++;
		g_bSerialSendingByte = true;
		p_tDataLength--;
	}
	// ���� �� ���� ���� ��� output buffer ������ �� ����������� �� ����� byte
	// ��� �� �������� ��� �� ������. ���� ��� ���� l_tpTemp.
	// ��� �������� ��������, ������ ������ ��� ���������� ��� ram ��� ���� �������
	// 4 kbytes, ��� �� ������ ���� overflow.
	l_tpTemp = g_taSerialOutputBuffer + g_tSerialOutputBufferFirstDataByteIndex + g_tSerialOutputBufferDataSize;
	while (l_tpTemp >= g_taSerialOutputBuffer + SERIAL_OUTPUT_BUFFER_SIZE)
	{
		l_tpTemp -= SERIAL_OUTPUT_BUFFER_SIZE;
	}
	while (p_tDataLength > 0)
	{
		*l_tpTemp = *p_tpData;
		p_tpData++;
		g_tSerialOutputBufferDataSize++;
		l_tpTemp++;
		if (l_tpTemp >= g_taSerialOutputBuffer + SERIAL_OUTPUT_BUFFER_SIZE)
		{
			l_tpTemp = g_taSerialOutputBuffer;
		}
		p_tDataLength--;
	}
	// ������� ���� output buffer �� �������� ���. ��������� ��� ��� ���������
	// ��� interrupts ���� ��� ����.
	SREG = l_tStatusRegister;
}


void  SerialWriteByte(uint8_t p_tData)
{
	SerialWriteData(&p_tData, 1);
}


// ���������� ��� ������ ��� bytes ��� ����� ��������� ���� ������� ��������
// buffer ������.
uint16_t SerialGetOutputBufferFreeSpace()
{
	uint8_t l_tStatusRegister;
	uint16_t l_rReturnValue;
	
	l_tStatusRegister = SREG;
	cpu_irq_disable();
	l_rReturnValue = SERIAL_OUTPUT_BUFFER_SIZE - g_tSerialOutputBufferDataSize;
	SREG = l_tStatusRegister;
	return l_rReturnValue;
}


// ��������� ��� �������� ������ �� �������� ���� ������� �������� buffer �������.
void SerialClearInputBuffer()
{
	uint8_t l_tStatusRegister;
	
	l_tStatusRegister = SREG;
	cpu_irq_disable();
	g_tSerialInputBufferDataSize = 0;
	g_tSerialInputBufferFirstDataByteIndex = 0;
	SREG = l_tStatusRegister;
}


// ��������� ��� �������� ������ �� �������� ���� ������� �������� buffer ������.
void SerialClearOutputBuffer()
{
	uint8_t l_tStatusRegister;
	
	l_tStatusRegister = SREG;
	cpu_irq_disable();
	g_tSerialOutputBufferDataSize = 0;
	g_tSerialOutputBufferFirstDataByteIndex = 0;
	// ��� ���������� ��� g_bSerialSendingByte. �� ���� �� ������ �������
	// �������� ���������, �� �� ����� ��� ���������, � ������ �� ���������
	// ��� ��� interrupt handler, ��� ���� ��� �� ������ ������ ���� ���
	// ��������.
	SREG = l_tStatusRegister;
}


// ���������� true �� ���� ������������ ��������� � ���������� �������� (�� ����
// ����� ��� �� ��������� bit ��� �� �������).
bool SerialIsTransmitionComplete()
{
	uint8_t l_tStatusRegister;
	bool l_bReturnValue;
	
	l_tStatusRegister = SREG;
	cpu_irq_disable();
	l_bReturnValue = g_tSerialOutputBufferDataSize == 0 && !g_bSerialSendingByte;
	SREG = l_tStatusRegister;
	return l_bReturnValue;
}


////////////////////////////////////////////////////////////////////////////////
// Interrupt services.
////////////////////////////////////////////////////////////////////////////////


// Interrupt service ��� �� USART0 receive interrupt.
ISR(USART_RX_vect)
{
	uint8_t l_tUCSRA;
	uint8_t l_tUDR;

	// ������� ��� ���������� ��� �������� �� ������ parity error.
	l_tUCSRA = UCSR0A;
	l_tUDR = UDR0;
	// ������������ �� byte ��� ��������� ���� �� ��� ������ parity error.
	if ((l_tUCSRA & _BV(UPE0)) == 0)
	{
		// �� ������� ���������� ����� ���� input buffer, ����������.
		// ������ ��� ��� ���� �������.
		if (g_tSerialInputBufferDataSize < SERIAL_INPUT_BUFFER_SIZE)
		{
			// ����� �� ���� offset �� ������ �� ������������� ��� ���������
			// ��� ������. ��� �������� ��������, ������ ������ ��� ���������� ��� ram
			// ��� ���� ������� 4 kbytes, ��� �� ������ ���� overflow.
			uint8_t *l_tpTemp;
			l_tpTemp = g_taSerialInputBuffer + g_tSerialInputBufferFirstDataByteIndex + g_tSerialInputBufferDataSize;
			while (l_tpTemp >= g_taSerialInputBuffer + SERIAL_INPUT_BUFFER_SIZE)
			{
				l_tpTemp -= SERIAL_INPUT_BUFFER_SIZE;
			}
			*l_tpTemp = l_tUDR;
			// �������� ��� ������ ��� byte ��������.
			g_tSerialInputBufferDataSize++;
		}
	}
}


// Interrupt service ��� �������� ��� �� �������� ���� 0 (USART0 transmit
// interrupt). ��� ��������� ���� ���� ����� � ���������� ��� ��� ���������
// ������ (�� interrupt ��� � transmit buffer ����� ��������� ����� �� USART_UDRE_vect).
ISR(USART_TX_vect)
{
	// ������ ��� ���� ���� UDR ���� �����, �������� ���� ��������� ���������.
	g_bSerialSendingByte = false;
    // ����� �� ������� ������ ���� ���� output buffer.
    if (g_tSerialOutputBufferDataSize > 0)
    {
		// ���� �������� ���� UDR0, ����� ����������� �� ������ bit (�� ��������� ���
		// ������ ����� 9-bits).
		UCSR0B &= ~_BV(TXB80);
		UDR0 = g_taSerialOutputBuffer[g_tSerialOutputBufferFirstDataByteIndex];
		// �������� ��� ������� ���������� ��� ������ ��� ��� UDR.
		g_bSerialSendingByte = true;
		// ��������� ��������� ��� ���������� ��� ������.
		g_tSerialOutputBufferDataSize--;
		// �� ��� ������� ������ ���� �� �����, �������� ���
		// g_rSerialOutputBufferFirstDataByteIndex ��� ������� ��������.
		if (g_tSerialOutputBufferDataSize == 0)
		{
			g_tSerialOutputBufferFirstDataByteIndex = 0;
		}
		else
		{
			g_tSerialOutputBufferFirstDataByteIndex++;
			if (g_tSerialOutputBufferFirstDataByteIndex >= SERIAL_OUTPUT_BUFFER_SIZE)
				g_tSerialOutputBufferFirstDataByteIndex = 0;
		}
    }
}

