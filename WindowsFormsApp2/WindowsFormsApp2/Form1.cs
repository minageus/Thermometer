using System;
using System.IO.Ports;
using System.Windows.Forms;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using System.Threading;
using System.Diagnostics;
using System.Collections;

namespace WindowsFormsApp2
{
    public partial class Form1 : Form
    {
        string ReadResult = string.Empty;
        string SerialPortName = string.Empty;
        // Ότι δεδομένα δέχεται η serialPort_DataReceived, στέλνονται με BeginInvoke στην
        // SerialDataReceivedHandler, η οποία τα κάνει append στην serialReceiveBuffer
        // και κατόπιν τα εξυπηρετεί από εκεί.
        private readonly List<byte> serialReceiveBuffer = new List<byte>();
        private delegate void SerialDataReceived(byte[] data);
        static int commtickcounter;
        


        enum CommState
        {
            NULL,
            portclosed,
            sentrc,
            sentwc
        }

        private CommState state;

        void NextState ()
        {
            string methodName = state.ToString();
            System.Reflection.MethodInfo info =
                GetType().GetMethod(methodName, System.Reflection.BindingFlags.NonPublic |
                System.Reflection.BindingFlags.Instance);
            StartCoroutine((IEnumerator)info.Invoke(this, null));
        }

        private void StartCoroutine(IEnumerator enumerator)
        {
            throw new NotImplementedException();
        }

        IEnumerator NULLState()
        {
            Debug.WriteLine("NULL STATE\n");
            while (state == CommState.NULL)
                yield return 0;
            commtickcounter++;
            NextState();
        }

        IEnumerator PortClosedState ()
        {
            Debug.WriteLine("PORTCLOSED STATE");
            while(state == CommState.portclosed)
            {
                yield return 0;
            }
            commtickcounter++;
            NextState();
        }

        IEnumerator sentrcState ()
        {
            Debug.WriteLine("SENTRC STATE");
            while(state == CommState.sentrc)
            {
                int bytesToRead = this.serialPort1.BytesToRead;
                byte[] readBuffer = new byte[bytesToRead];
                int bytesRead = this.serialPort1.Read(readBuffer, 0, bytesToRead);
                this.BeginInvoke(new SerialDataReceived(SerialDataReceivedHandler), readBuffer);
                yield return 0;
            }
            commtickcounter++;
            NextState();
        }

        IEnumerator sentwcState ()
        {
            Debug.WriteLine("SENTWC STATE");
            while(state == CommState.sentwc)
            {
                String stringToSend = numericUpDown5.Value.ToString() + " " + numericUpDown2.Value.ToString() +
            " " + numericUpDown3.Value.ToString() + " " + numericUpDown1.Value.ToString() + " " +
            numericUpDown4.Value.ToString();
                yield return 0;
            }
            commtickcounter++;
            NextState();
        }

        public Form1()
        {
            InitializeComponent();
        }
        
 
        private void Form1_Load(object sender, EventArgs e)
        {
            this.comboBox1.Items.AddRange(System.IO.Ports.SerialPort.GetPortNames());
            if (this.comboBox1.Items.Count == 0)
            {
                this.comboBox1.Items.Add("COM1");
            }
        }

   
        private void button4_Click(object sender, EventArgs e)
        {
            String stringToSend = numericUpDown5.Value.ToString() + " " + numericUpDown2.Value.ToString() +
            " " + numericUpDown3.Value.ToString() + " " + numericUpDown1.Value.ToString() + " " +
            numericUpDown4.Value.ToString();
                
            byte[] bytesToSend = Encoding.ASCII.GetBytes(stringToSend);
            this.serialPort1.Write(bytesToSend, 0, bytesToSend.Length);
        }


 
        private void button3_Click(object sender, EventArgs e)
        {
        }

       

        private void SerialDataReceivedHandler(byte[] dataReceived)
        {
            this.serialReceiveBuffer.AddRange(dataReceived);
            int indexOfLf = this.serialReceiveBuffer.FindIndex(x => x == 0x0a);
            if (indexOfLf >= 0)
            {
                var message = this.serialReceiveBuffer.GetRange(0, indexOfLf + 1);
                Debug.Write(Encoding.ASCII.GetString(message.ToArray()));
                this.serialReceiveBuffer.RemoveRange(0, indexOfLf + 1);
            }
        }

        private void bwRead_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            if(ReadResult.Length > 0)
            {
                MessageBox.Show("ERROR {0}\n", ReadResult);
            }
            else
            {
                MessageBox.Show("SUCCESS\n");
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            try
            {
                this.serialPort1.PortName = this.comboBox1.Text.Trim();
                this.serialPort1.Open();
            }
            catch (Exception)
            {
                // Error opening the serial port.
                return;
            }

        }

        private void button2_Click(object sender, EventArgs e)
        {
            try
            {
                this.serialPort1.Close();
            }
            catch (Exception)
            {
            }

        }

        private void serialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                if (e.EventType == System.IO.Ports.SerialData.Chars)
                {
                    int bytesToRead = this.serialPort1.BytesToRead;
                    byte[] readBuffer = new byte[bytesToRead];
                    int bytesRead = this.serialPort1.Read(readBuffer, 0, bytesToRead);
                    this.BeginInvoke(new SerialDataReceived(SerialDataReceivedHandler), readBuffer);
                }
            }
            catch (Exception)
            {
            }

        }


        private void timer1_Tick(object sender, EventArgs e)
        {
            NULLState();
            PortClosedState();
            sentrcState();
            sentwcState();
            if(commtickcounter>=50)
            {
                MessageBox.Show("ERROR");
            }
        }
    }

}
