//
// The sample-push socket shared by the Flapjack SIL's emulated sensor parts.
//
// Every emulated part is fed the same way: bridge.py opens one TCP connection
// per part and streams fixed-length frames at whatever rate it likes, and the
// part serves the newest one through its own register map. That plumbing is
// identical for the IMU, the magnetometer and the barometer, so it lives here
// and the models carry only their register maps and their scaling.
//
// Frame: 0xAA 0x55 then payloadLength bytes, little endian. The header is
// resynchronised on every frame, so a client that connects mid-stream lands on
// a frame boundary instead of decoding garbage.
//
// The push is one-way and never blocks the emulation thread: the receive thread
// only ever swaps a reference and Latest only ever reads one, so an SPI
// transaction costs the same whether the socket is busy or idle.
//
// Load with `include @Scripts/renode/SamplePushListener.cs` BEFORE the models
// that use it - Renode loads each ad-hoc assembly as it compiles it, so a later
// include can reference this one, but not the other way round.
//
using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;

namespace Antmicro.Renode.Peripherals.Sensors
{
    // One received frame. Sequence is what lets a model answer "is this sample
    // one I have already served?", which is how the data-ready bits are driven.
    public sealed class PushedSample
    {
        public PushedSample(byte[] payload, long sequence)
        {
            Payload = payload;
            Sequence = sequence;
        }

        public readonly byte[] Payload;
        public readonly long Sequence;
    }

    public sealed class SamplePushListener : IDisposable
    {
        // port 0 leaves the socket off: the model is then just its register
        // file, which is all a boot smoke test needs.
        public SamplePushListener(IEmulationElement owner, int port, int payloadLength)
        {
            this.owner = owner;
            this.payloadLength = payloadLength;

            if(port == 0)
            {
                return;
            }

            try
            {
                listener = new TcpListener(IPAddress.Loopback, port);
                listener.Start();
            }
            catch(SocketException e)
            {
                // A stale Renode still holding the port must not stop the
                // machine from being created - without a source the model is
                // still a working register file.
                listener = null;
                owner.Log(LogLevel.Warning, "Could not listen on port {0}: {1}", port, e.Message);
                return;
            }

            receiver = new Thread(ReceiveLoop) { IsBackground = true, Name = "sample-push-" + port };
            receiver.Start();
            owner.Log(LogLevel.Info, "Listening for sample pushes on port {0}", port);
        }

        public void Dispose()
        {
            disposed = true;
            listener?.Stop();
            client?.Close();
        }

        // True when a socket was asked for and opened. Models use this to tell
        // "the bridge has not pushed yet" (report no data) from "there is no
        // bridge at all" (serve the register file), which are different answers.
        public bool Active
        {
            get { return listener != null; }
        }

        // Null until the first frame lands, and again once the client goes away.
        public PushedSample Latest
        {
            get { return Volatile.Read(ref latest); }
        }

        private void ReceiveLoop()
        {
            var payload = new byte[payloadLength];
            while(!disposed)
            {
                try
                {
                    client = listener.AcceptTcpClient();
                    client.NoDelay = true;
                    owner.Log(LogLevel.Info, "Sample source connected");

                    var stream = client.GetStream();
                    while(!disposed)
                    {
                        SyncToFrameStart(stream);
                        ReadExactly(stream, payload, 0, payloadLength);
                        Volatile.Write(ref latest, new PushedSample((byte[])payload.Clone(), ++pushed));
                    }
                }
                catch(Exception e) when(e is IOException || e is SocketException || e is ObjectDisposedException)
                {
                    if(!disposed)
                    {
                        owner.Log(LogLevel.Info, "Sample source disconnected: {0}", e.Message);
                    }
                }
                finally
                {
                    client?.Close();
                    client = null;
                    // Stop serving a sample nobody is refreshing any more; reads
                    // fall back to the register file rather than freezing on the
                    // last one the bridge happened to send.
                    Volatile.Write(ref latest, null);
                }
            }
        }

        private static void SyncToFrameStart(Stream stream)
        {
            var state = 0;
            while(state < 2)
            {
                var b = stream.ReadByte();
                if(b < 0)
                {
                    throw new IOException("Stream closed while looking for a frame header");
                }
                state = (b == FrameMagic0) ? 1 : ((state == 1 && b == FrameMagic1) ? 2 : 0);
            }
        }

        private static void ReadExactly(Stream stream, byte[] buffer, int offset, int count)
        {
            while(count > 0)
            {
                var read = stream.Read(buffer, offset, count);
                if(read <= 0)
                {
                    throw new IOException("Stream closed mid-frame");
                }
                offset += read;
                count -= read;
            }
        }

        private volatile bool disposed;
        private TcpListener listener;
        private TcpClient client;
        private Thread receiver;
        private long pushed;
        private PushedSample latest;

        private readonly IEmulationElement owner;
        private readonly int payloadLength;

        private const byte FrameMagic0 = 0xAA;
        private const byte FrameMagic1 = 0x55;
    }
}
