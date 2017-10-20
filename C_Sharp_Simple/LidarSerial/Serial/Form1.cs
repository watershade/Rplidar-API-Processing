using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
//using System.Threading;

namespace Serial
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
            InitForm();
        }

        //private uint[] baudRateList;
        private int lidarSpeed;
        private void InitForm() {
            /*set Serial port name*/
            foreach (string s in SerialPort.GetPortNames())
            {
                comboBox1.Items.Add(s);
            }
            /*if item in comboBox is more than 1,show it*/
            if (comboBox1.Items.Count > 0) {
                comboBox1.SelectedIndex = 0;
                SPLidar.PortName = comboBox1.SelectedItem.ToString();
                Console.WriteLine("Serial Port is " + SPLidar.PortName);
            }

            comboBox2.Items.AddRange(new object[] { 4800,9600,19200,38400,115200,460800});
            comboBox2.SelectedIndex = 4;
            /*initialize lidarSpeed */
            lidarSpeed = 800;
            if (lidarSpeed > 1024) lidarSpeed = 1024;
            trackBar1.Value = lidarSpeed;
            textBox1.Text = lidarSpeed.ToString();


            InitSPLidar();
        }

        private void InitSPLidar() {
            SPLidar.BaudRate = 115200;
            SPLidar.StopBits = System.IO.Ports.StopBits.One;
            SPLidar.Parity = System.IO.Ports.Parity.None;
        }
        private void button1_Click(object sender, EventArgs e)
        {
            if (SPLidar.IsOpen) return;
            SPLidar.Open();
            button2.Enabled = true;
            button1.Enabled = false;
        }

        private void Form1_Load(object sender, EventArgs e)
        {

        }

        private void button3_Click(object sender, EventArgs e)
        {
            byte[] CmdBuf = { 0xa5, 0xf0, 0x02, 0x58, 0x02, 0x0d };

            CmdBuf[5] = (0xa5 ^ 0xf0 ^ 0x02);
            CmdBuf[3] = (byte)(lidarSpeed & 0xff);
            CmdBuf[5] ^= CmdBuf[3];
            CmdBuf[4] = (byte)(lidarSpeed>>8 & 0xff);
            CmdBuf[5] ^= CmdBuf[4];

            if (SPLidar.IsOpen)
            {
                SPLidar.Write(CmdBuf, 0, 6);
            }
        }

        private void stopScan()
        {
            byte[] CmdBuf = { 0xa5, 0x25 };
            if (SPLidar.IsOpen)
            {
                SPLidar.Write(CmdBuf, 0, 2);
            }
        }
        private void button4_Click(object sender, EventArgs e)
        {
            stopScan();
        }

        private void label1_Click(object sender, EventArgs e)
        {

        }

        private void button5_Click(object sender, EventArgs e)
        {
            byte[] CmdBuf = { 0xa5, 0xf0, 0x02, 0x00, 0x00, 0x57 };
            if (SPLidar.IsOpen)
            {
                SPLidar.Write(CmdBuf, 0, 6);
            }
        }

        private void button2_Click(object sender, EventArgs e)
        {
            if (SPLidar.IsOpen) {
                SPLidar.Close();
                button1.Enabled = true;
                button2.Enabled = false;
            } 
        }

        private void trackBar1_BindingContextChanged(object sender, EventArgs e)
        {
        }

        private void trackBar1_ValueChanged(object sender, EventArgs e)
        {
            lidarSpeed = trackBar1.Value;
            Console.WriteLine("Lidar speed is " + lidarSpeed);
            textBox1.Text = trackBar1.Value.ToString();
        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {
            lidarSpeed = Convert.ToInt32(textBox1.Text);
            trackBar1.Value = lidarSpeed;
            Console.WriteLine("Lidar speed is " + lidarSpeed);
        }

        private void comboBox1_SelectedIndexChanged(object sender, EventArgs e)
        {
            SPLidar.PortName = comboBox1.SelectedItem.ToString();
        }

        private void comboBox2_SelectedIndexChanged(object sender, EventArgs e)
        {
            SPLidar.BaudRate = Convert.ToInt32(comboBox2.SelectedItem.ToString());
            Console.WriteLine("Lidar baudrate is " + SPLidar.BaudRate);
        }
    }
}
