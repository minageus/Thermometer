////////////////////////////////////////////////////////////////////////////////
// Module του προγράμματος QN1610GT που περιλαμβάνει τις συναρτήσεις
// σειριακής επικοινωνίας.
// Δουλεύουμε μόνο με το USART0.
////////////////////////////////////////////////////////////////////////////////

// Το header του τρέχοντος module (περιλαμβάνει το βασικό header BaseHeader.h).
#include "Serial.h"

// Τα υπόλοιπα headers που χρειαζόμαστε.


////////////////////////////////////////////////////////////////////////////////
// Global δεδομένα για τη σειριακή επικοινωνία.
// Επειδή το πρωτόκολλο της συσκευής ΤΝΕ6 έχει μικρές εγγραφές, χρησιμοποιούμε
// uint8_t για μετρητές.
////////////////////////////////////////////////////////////////////////////////

// Οι κυκλικοί buffers για είσοδο και έξοδο χαρακτήρων.
uint8_t g_taSerialInputBuffer[SERIAL_INPUT_BUFFER_SIZE];
uint8_t g_taSerialOutputBuffer[SERIAL_OUTPUT_BUFFER_SIZE];

// Πόσα πραγματικά δεδομένα υπάρχουν μέσα στον buffer εισόδου.
// Επιτρέπεται να πάρει τιμή έως και SERIAL_INPUT_BUFFER_SIZE.
uint8_t g_tSerialInputBufferDataSize = 0;
// Index μέσα στον buffer εισόδου που δείχνει που είναι το πρώτο byte
// που ήλθε.
uint8_t g_tSerialInputBufferFirstDataByteIndex = 0;

// Πόσα πραγματικά δεδομένα υπάρχουν μέσα στον buffer εξόδου.
// Επιτρέπεται να πάρει τιμή έως και SERIAL_OUTPUT_BUFFER_SIZE.
uint8_t g_tSerialOutputBufferDataSize = 0;
// Index μέσα στον buffer εξόδου που δείχνει που είναι το πρώτο byte
// που είναι για να φύγει στο σειριακό.
uint8_t g_tSerialOutputBufferFirstDataByteIndex = 0;

// Αν καλέσουμε την WriteData να στείλει ένα byte και δεν έχουμε δεδομένα
// στον buffer, η WriteData στέλνει το byte απ' ευθείας στο UDR. Αν καλέσουμε
// μετά αμέσως την WriteData, αυτή δεν θα ξέρει ότι αυτή τη στιγμή
// στέλνονται δεδομένα και θα ξαναγράψει στον UDR. Βάζουμε λοιπόν τη μεταβλητή
// m_bSerialSendingByte για να κρατάει λογαριασμό αν κάποια στιγμή γίνεται
// αποστολή.
bool g_bSerialSendingByte = false;


////////////////////////////////////////////////////////////////////////////////
// Συναρτήσεις του module.
////////////////////////////////////////////////////////////////////////////////


// Συνάρτηση με την οποία αρχικοποιούμε τη σειριακή επικοινωνία.
// Ρυθμίζει και το USART και τις μεταβλητές μας.
// Η παράμετρος p_rUbrr περιέχει τον διαιρέτη που θα γράψουμε στον καταχωρητή
// UBRR για να θέσουμε το baud rate.
void SerialInitialize(uint16_t p_rUbrr)
{
	uint8_t l_tStatusRegister;
	
	l_tStatusRegister = SREG;
	cpu_irq_disable();
	// Αν η σειριακή εκπομπή είναι ήδη ενεργοποιημένη, θα πρέπει να περιμένουμε
	// να τελειώσει (έτσι λέει το datasheet του AVR) πριν του αλλάξουμε baud rate.
	// Βέβαια εμείς για να αλλάξουμε το baud rate κλείνουμε πρώτα εντελώς το σειριακό,
	// αλλά δεν πειράζει, καλό είναι να περιμένουμε να τελειώσει η όποια εκπομπή.
	if (UCSR0B & _BV(TXEN0))
	{
		cpu_irq_enable();
		while (!SerialIsTransmitionComplete());
		cpu_irq_disable();
	}
	// Απενεργοποιούμε εντελώς το σειριακό, θέτοντας τις τιμές που ισχύουν στο reset.
	// Γράφουμε πρώτα τον UCSR0B που κλείνει το σειριακό.
	UCSR0B = 0x00;
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	// Αρχικοποιούμε τις μεταβλητές μας.
	g_tSerialInputBufferDataSize = 0;
	g_tSerialInputBufferFirstDataByteIndex = 0;
	g_tSerialOutputBufferDataSize = 0;
	g_tSerialOutputBufferFirstDataByteIndex = 0;
	g_bSerialSendingByte = false;

	// Θέτουμε το baud rate.
	// Γράφοντας απ' ευθείας στον UBRR0 (16-bit καταχωρητής), ο compiler βάζει σωστά
	// να γράφεται πρώτα ο UBRR0H και μετά ο UBRR0L.
	UBRR0 = p_rUbrr;
	// Η παραπάνω αρχικοποίηση των καταχωρητών του USART0 έχει θέσει 8 bits, 1 stop bit,
	// no parity, που είναι τα χαρακτηριστικά που θέλουμε. Έτσι δεν χρειαζόμαστε κάποια
	// περαιτέρω αρχικοποίηση.
    // Ενεργοποιούμε την εκπομπή και τη λήψη μαζί με τα interrupt τους.
	UCSR0B = _BV(RXCIE0) | _BV(TXCIE0) | _BV(RXEN0) | _BV(TXEN0);
	SREG = l_tStatusRegister;
}


// Συνάρτηση που διαβάζει ένα byte από τον κυκλικό σειριακό buffer εισόδου,
// τον βάζει στην p_tpData και επιστρέφει true. Αν δεν υπάρχει byte διαθέσιμο,
// επιστρέφει false.
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


// Συνάρτηση που βάζει δεδομένα στον κυκλικό σειριακό buffer εξόδου.
// Αν ο buffer εξόδου δεν έχει αρκετό χώρο, περιμένει μέχρι να έχει.
void SerialWriteData(uint8_t *p_tpData, uint8_t p_tDataLength)
{
	uint8_t l_tStatusRegister;
	uint8_t *l_tpTemp;
	
	if (p_tDataLength == 0)
	{
		return;
	}
	// Αν τα δεδομένα που θέλουμε να στείλουμε είναι περισσότερα από το μέγιστο
	// χώρο του buffer, φύγε.
	if (p_tDataLength > SERIAL_OUTPUT_BUFFER_SIZE)
	{
		return;
	}
	// Πρώτα περίμενε να έχει αρκετό χώρο ο buffer εξόδου. Παρόλο που την μεταβλητή
	// αυτή την αλλάζουν και οι συναρτήσεις interrupt, μπορούμε να την ελέγχουμε
	// ελεύθερα γιατί έχει μήκος ένα byte.
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
	// Εδώ έχουμε στον output buffer αρκετό χώρο για να στείλουμε τα δεδομένα μας.
	// Εδώ που μόλις έχουμε απενεργοποιήσει τα interrupts, και δεν υπάρχει περίπτωση
	// να κληθεί η συνάρτηση interrupt που στέλνει στο σειριακό, κοίτα αν είναι
	// false η m_bSerialSendingByte. Αν ναι, τότε δεν υπάρχει χαρακτήρας που στέλνεται,
	// οπότε πρέπει να βάλουμε τον πρώτο χαρακτήρα να φεύγει, και να φυλάξουμε
	// τους άλλους στον output buffer, αλλιώς τους βάζουμε όλους στον buffer χωρίς
	// να στείλουμε στο UDR.
	if (!g_bSerialSendingByte)
	{
		// Όταν γράφουμε στον UDR0, πρώτα μηδενίζουμε το έννατο bit (σε περίπτωση που
		// έχουμε χρήση 9-bits).
		UCSR0B &= ~_BV(TXB80);
		UDR0 = *p_tpData;
		p_tpData++;
		g_bSerialSendingByte = true;
		p_tDataLength--;
	}
	// Βρες σε ποιά θέση του output buffer πρέπει να αποθηκευθεί το πρώτο byte
	// από τα δεδομένα που θα φύγουν. Βάλε την στην l_tpTemp.
	// Στο παρακάτω άθροισμα, εφόσον μιλάμε για μεταβλητές στη ram που έχει μέγεθος
	// 4 kbytes, δεν θα έχουμε ποτέ overflow.
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
	// Γράψαμε στον output buffer τα δεδομένα μας. Επανάφερε και την κατάσταση
	// των interrupts εκεί που ήταν.
	SREG = l_tStatusRegister;
}


void  SerialWriteByte(uint8_t p_tData)
{
	SerialWriteData(&p_tData, 1);
}


// Επιστρέφει τον αριθμό των bytes που είναι διαθέσιμα στον κυκλικό σειριακό
// buffer εξόδου.
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


// Καθαρίζει ότι δεδομένα μπορεί να υπάρχουν στον κυκλικό σειριακό buffer εισόδου.
void SerialClearInputBuffer()
{
	uint8_t l_tStatusRegister;
	
	l_tStatusRegister = SREG;
	cpu_irq_disable();
	g_tSerialInputBufferDataSize = 0;
	g_tSerialInputBufferFirstDataByteIndex = 0;
	SREG = l_tStatusRegister;
}


// Καθαρίζει ότι δεδομένα μπορεί να υπάρχουν στον κυκλικό σειριακό buffer εξόδου.
void SerialClearOutputBuffer()
{
	uint8_t l_tStatusRegister;
	
	l_tStatusRegister = SREG;
	cpu_irq_disable();
	g_tSerialOutputBufferDataSize = 0;
	g_tSerialOutputBufferFirstDataByteIndex = 0;
	// Δεν πειράζουμε την g_bSerialSendingByte. Αν αυτή τη στιγμή γίνεται
	// αποστολή χαρακτήρα, με το τέλος της αποστολής, η σημαία θα καθαρίσει
	// από τον interrupt handler, και μετά δεν θα βρεθεί τίποτε άλλο για
	// αποστολή.
	SREG = l_tStatusRegister;
}


// Επιστρέφει true αν έχει ολοκληρωτικά τελειώσει η διαδικασία εκπομπής (να έχει
// φύγει και το τελευταίο bit από το καλώδιο).
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


// Interrupt service για το USART0 receive interrupt.
ISR(USART_RX_vect)
{
	uint8_t l_tUCSRA;
	uint8_t l_tUDR;

	// Διάβασε τον καταχωρητή που περιέχει τη σημαία parity error.
	l_tUCSRA = UCSR0A;
	l_tUDR = UDR0;
	// Αποθηκεύουμε το byte που δεχθήκαμε μόνο αν δεν έχουμε parity error.
	if ((l_tUCSRA & _BV(UPE0)) == 0)
	{
		// Αν υπάρχει διαθέσιμος χώρος στον input buffer, αποθήκευσε.
		// Αλλιώς ότι μας ήλθε χάνεται.
		if (g_tSerialInputBufferDataSize < SERIAL_INPUT_BUFFER_SIZE)
		{
			// Κοίτα σε ποιό offset θα πρέπει να αποθηκεύσουμε τον χαρακτήρα
			// που λάβαμε. Στο παρακάτω άθροισμα, εφόσον μιλάμε για μεταβλητές στη ram
			// που έχει μέγεθος 4 kbytes, δεν θα έχουμε ποτέ overflow.
			uint8_t *l_tpTemp;
			l_tpTemp = g_taSerialInputBuffer + g_tSerialInputBufferFirstDataByteIndex + g_tSerialInputBufferDataSize;
			while (l_tpTemp >= g_taSerialInputBuffer + SERIAL_INPUT_BUFFER_SIZE)
			{
				l_tpTemp -= SERIAL_INPUT_BUFFER_SIZE;
			}
			*l_tpTemp = l_tUDR;
			// Σημείωσε ότι έχουμε ένα byte επιπλέον.
			g_tSerialInputBufferDataSize++;
		}
	}
}


// Interrupt service για αποστολή από τη σειριακή θύρα 0 (USART0 transmit
// interrupt). Εδώ ερχόμαστε όταν έχει φύγει ο χαρακτήρας από τον ακροδέκτη
// εξόδου (το interrupt που ο transmit buffer είναι ελεύθερος είναι το USART_UDRE_vect).
ISR(USART_TX_vect)
{
	// Εφόσον ότι ήταν στον UDR έχει φύγει, σημείωσε στην κατάλληλη μεταβλητή.
	g_bSerialSendingByte = false;
    // Κοίτα αν υπάρχει τίποτε άλλο στον output buffer.
    if (g_tSerialOutputBufferDataSize > 0)
    {
		// Όταν γράφουμε στον UDR0, πρώτα μηδενίζουμε το έννατο bit (σε περίπτωση που
		// έχουμε χρήση 9-bits).
		UCSR0B &= ~_BV(TXB80);
		UDR0 = g_taSerialOutputBuffer[g_tSerialOutputBufferFirstDataByteIndex];
		// Σημείωσε ότι υπάρχει χαρακτήρας που φεύγει από τον UDR.
		g_bSerialSendingByte = true;
		// Ενημέρωσε κατάλληλα τις μεταβλητές της κλάσης.
		g_tSerialOutputBufferDataSize--;
		// Αν δεν υπάρχει τίποτε άλλο να φύγει, μηδένισε την
		// g_rSerialOutputBufferFirstDataByteIndex για γρήγορο χειρισμό.
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

