﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net;
using System.Net.Sockets;
using System.IO;
using System.Text.RegularExpressions;
using NetJobTransfer;

namespace TESTHerdAgent
{
    //enum AgentState { BUSY, AVAILABLE, DISCOVERED };
    enum FileType { EXE, INPUT, OUTPUT};

    class Program
    {

        static AgentState m_state = AgentState.AVAILABLE;
        static HerdAgent herdAgent = new HerdAgent();




      

        public static void DiscoveryCallback(IAsyncResult ar)
        {

            UdpClient u = (UdpClient)((UdpState)(ar.AsyncState)).u;
            IPEndPoint e = (IPEndPoint)((UdpState)(ar.AsyncState)).e;


            Byte[] receiveBytes = u.EndReceive(ar, ref e);
            string receiveString = Encoding.ASCII.GetString(receiveBytes);

            
           

            if (receiveString == "QUIT")
            {
                if(Program.m_state==AgentState.BUSY)
                {
                    m_state = AgentState.CANCELING;
                    herdAgent.stop();
                 
                    m_state = AgentState.AVAILABLE;
                }
                
               
            }
            else if(Program.m_state==AgentState.AVAILABLE)
            {
               
                byte[] data = Encoding.ASCII.GetBytes("<Cores>" + Environment.ProcessorCount + "</Cores>");
                u.Send(data, data.Length, e);
                Program.m_state = AgentState.DISCOVERED;
            }
            

            u.BeginReceive(new AsyncCallback(DiscoveryCallback), ar.AsyncState);
        }
        static void Main(string[] args)
        {
           
            UdpClient m_discoverySocket;
            TcpClient m_comSocket=null;
               
            m_discoverySocket = new UdpClient(CJobDispatcher.m_discoveryPortHerd);
            UdpState state = new UdpState();
            IPEndPoint shepherd= new IPEndPoint(0,0);
            UdpState u = new UdpState();
            u.e = shepherd;
            u.u = m_discoverySocket;
            u.c = herdAgent;
           
            m_discoverySocket.BeginReceive(DiscoveryCallback,u);

            while (true)
            {
                if (Program.m_state == AgentState.DISCOVERED)
                {

                    Program.m_state = AgentState.BUSY;
                    var server = new TcpListener(IPAddress.Any, 4444);
                    server.Start();
                    m_comSocket = server.AcceptTcpClient();
                    NetworkStream netStream = m_comSocket.GetStream();
                    byte[] doIHaveToWork = new byte[24];
                    netStream.Read(doIHaveToWork, 0, 24);
                    if (Encoding.ASCII.GetString(doIHaveToWork).Split('\n')[0].Equals("You are free"))
                    {

                    }
                    else
                    {
                        herdAgent = new HerdAgent();
                        if (herdAgent.ReceiveJobQuery(netStream))
                        {
                            herdAgent.RunJob(netStream);
                            byte[] stopM = new byte[256];
                            for (int i = 0; i < stopM.Length; i++)
                                stopM[i] = 32;
                            byte [] tmp=Encoding.ASCII.GetBytes("There is no more data");
                            Array.Copy(tmp, stopM, tmp.Length);

                                
                            netStream.Write(stopM, 0, stopM.Length);
                            herdAgent.SendJobResult(netStream);
                        }
                    }
                    netStream.Close();
                    netStream.Dispose();
                    server.Stop();
                    m_comSocket.Close();
                    m_state = AgentState.AVAILABLE;

                }
            }

        }


    }
}