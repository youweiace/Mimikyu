using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Xml.Linq;

using Grasshopper.Kernel;
using Mimikyu.Helper;
using Rhino;
using Rhino.Geometry;

namespace Mimikyu.Utlilities
{
    public class GH_Server : GH_Component
    {
        private const int Port = 59152;
        private readonly ConcurrentQueue<RobotPose> poses = new ConcurrentQueue<RobotPose>();
        private readonly object listenerLock = new object();
        private readonly object statusLock = new object();
        private readonly object poseChangeLock = new object();
        private CancellationTokenSource cancellationSource;
        private Task listenerTask;
        private TcpListener activeListener;
        private bool isListening;
        private long poseCount;
        private string latestStatus = string.Empty;
        private RobotPose lastPose;
        private int positionChanged;

        /// <summary>
        /// Initializes a new instance of the GH_Server class.
        /// </summary>
        public GH_Server()
          : base("Server", "S",
              "PC as server listening to robot actions",
              "Mimikyu", "Utilites")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Listen", "L", "Start/stop the TCP listener.", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Reset", "R", "Clear received data.", GH_ParamAccess.item, false);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Poses", "P", "Received robot poses.", GH_ParamAccess.list);
            pManager.AddIntegerParameter("PoseCount", "C", "Total received pose count.", GH_ParamAccess.item);
            pManager.AddTextParameter("Status", "S", "Latest connection/status message.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("EventId", "PC", "Change number when the pose changes beyond tolerance.", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var listen = false;
            if (!DA.GetData(0, ref listen))
            {
                listen = false;
            }

            var reset = false;
            if (DA.GetData(1, ref reset) && reset)
            {
                ResetData();
            }

            EnsureListenerTaskState();
            UpdateListenerState(listen);
            DA.SetDataList(0, poses.ToArray());
            DA.SetData(1, (int)Math.Min(int.MaxValue, Interlocked.Read(ref poseCount)));
            DA.SetData(2, GetLatestStatus());
            DA.SetData(3, GetPositionChanged());
        }

        public override void RemovedFromDocument(GH_Document document)
        {
            StopListener();
            base.RemovedFromDocument(document);
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("99E9DE24-A9DA-4C43-AD14-A13ECAC53C1D"); }
        }

        private void StartListener()
        {
            if (listenerTask != null)
            {
                return;
            }

            cancellationSource = new CancellationTokenSource();
            listenerTask = Task.Run(() => ListenLoop(cancellationSource.Token));
            isListening = true;
            LogServerBanner();
        }

        private void StopListener()
        {
            if (listenerTask == null)
            {
                return;
            }

            try
            {
                LogMessage("[SHUTDOWN] Server stopping.");
                cancellationSource.Cancel();
                StopActiveListener();
                listenerTask.Wait(TimeSpan.FromSeconds(2));
            }
            catch (Exception ex)
            {
                LogError(ex);
            }
            finally
            {
                cancellationSource.Dispose();
                cancellationSource = null;
                listenerTask = null;
                isListening = false;
            }
        }

        private void UpdateListenerState(bool listen)
        {
            if (listen && !isListening)
            {
                StartListener();
            }
            else if (!listen && isListening)
            {
                StopListener();
            }
        }

        private void ListenLoop(CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                TcpListener listener = null;

                try
                {
                    listener = new TcpListener(IPAddress.Any, Port);
                    listener.Server.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ExclusiveAddressUse, false);
                    listener.Server.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                    listener.Start();
                    SetActiveListener(listener);
                    LogMessage("Waiting for robot connection ...");

                    while (!token.IsCancellationRequested)
                    {
                        if (!listener.Pending())
                        {
                            Thread.Sleep(50);
                            continue;
                        }

                        var client = listener.AcceptTcpClient();
                        var remote = client.Client.RemoteEndPoint as IPEndPoint;
                        if (remote != null)
                        {
                            LogMessage($"[CONNECTED] Robot at {remote.Address}:{remote.Port}");
                        }

                        using (client)
                        using (var stream = client.GetStream())
                        {
                            stream.ReadTimeout = Timeout.Infinite;
                            stream.WriteTimeout = 2000;
                            ReadClientStream(stream, token, remote);
                        }

                        LogMessage("Waiting for next robot connection ...");
                    }
                }
                catch (SocketException ex)
                {
                    if (ex.SocketErrorCode == SocketError.AccessDenied)
                    {
                        LogMessage("[ACCESS DENIED] Unable to bind to the port. Check firewall rules, port reservations, or run with elevated permissions.");
                        cancellationSource?.Cancel();
                        return;
                    }

                    LogError(ex);
                }
                catch (Exception ex)
                {
                    LogError(ex);
                }
                finally
                {
                    try
                    {
                        listener?.Stop();
                    }
                    catch (Exception ex)
                    {
                        LogError(ex);
                    }
                    finally
                    {
                        ClearActiveListener(listener);
                    }
                }

                Thread.Sleep(200);
            }
        }

        private void EnsureListenerTaskState()
        {
            if (listenerTask == null)
            {
                return;
            }

            if (!listenerTask.IsCompleted)
            {
                return;
            }

            try
            {
                cancellationSource?.Dispose();
            }
            catch
            {
            }

            cancellationSource = null;
            listenerTask = null;
            isListening = false;
        }

        private void ReadClientStream(NetworkStream stream, CancellationToken token, IPEndPoint remote)
        {
            var buffer = new byte[4096];
            var builder = new StringBuilder();

            while (!token.IsCancellationRequested && stream.CanRead)
            {
                int bytesRead;

                try
                {
                    bytesRead = stream.Read(buffer, 0, buffer.Length);
                }
                catch (IOException ex)
                {
                    if (!token.IsCancellationRequested)
                    {
                        LogError(ex);
                    }

                    if (remote != null)
                    {
                        LogMessage($"[DISCONNECTED] {remote.Address}:{remote.Port}");
                    }

                    break;
                }
                catch (Exception ex)
                {
                    LogError(ex);
                    break;
                }

                if (bytesRead <= 0)
                {
                    if (remote != null)
                    {
                        LogMessage($"[DISCONNECTED] {remote.Address}:{remote.Port}");
                    }

                    break;
                }

                builder.Append(Encoding.UTF8.GetString(buffer, 0, bytesRead));
                ParseBuffer(builder);
            }
        }

        private void ParseBuffer(StringBuilder builder)
        {
            while (true)
            {
                var current = builder.ToString();
                var startIndex = current.IndexOf("<FLANGE", StringComparison.OrdinalIgnoreCase);
                if (startIndex < 0)
                {
                    builder.Clear();
                    builder.Append(current);
                    break;
                }

                var endIndex = current.IndexOf("</FLANGE>", startIndex, StringComparison.OrdinalIgnoreCase);
                if (endIndex < 0)
                {
                    builder.Clear();
                    builder.Append(current.Substring(startIndex));
                    break;
                }

                var xml = current.Substring(startIndex, endIndex - startIndex + "</FLANGE>".Length);
                if (TryParsePose(xml, out var pose))
                {
                    poses.Enqueue(pose);
                    Interlocked.Increment(ref poseCount);
                    UpdatePoseChange(pose);
                    LogPose(pose);
                    NotifySolution();
                }

                builder.Clear();
                builder.Append(current.Substring(endIndex + "</FLANGE>".Length));
            }
        }

        private bool TryParsePose(string xml, out RobotPose pose)
        {
            pose = null;

            try
            {
                var document = XDocument.Parse(xml);
                var root = document.Root;
                if (root == null)
                {
                    return false;
                }

                pose = new RobotPose
                {
                    X = ReadDouble(root, "X"),
                    Y = ReadDouble(root, "Y"),
                    Z = ReadDouble(root, "Z"),
                    A = ReadDouble(root, "A"),
                    B = ReadDouble(root, "B"),
                    C = ReadDouble(root, "C"),
                    E1 = ReadDouble(root, "E1"),
                    E2 = ReadDouble(root, "E2"),
                    E3 = ReadDouble(root, "E3"),
                    E4 = ReadDouble(root, "E4"),
                    Timestamp = DateTime.UtcNow
                };

                return true;
            }
            catch (Exception ex)
            {
                LogError(ex);
                return false;
            }
        }

        private static double ReadDouble(XElement root, string name)
        {
            var element = root.Element(name);
            if (element == null)
            {
                return 0.0;
            }

            return double.TryParse(element.Value, NumberStyles.Float, CultureInfo.InvariantCulture, out var value)
                ? value
                : 0.0;
        }


        private void NotifySolution()
        {
            try
            {
                RhinoApp.InvokeOnUiThread(new Action(() =>
                {
                    var document = OnPingDocument();
                    if (document != null)
                    {
                        document.ScheduleSolution(1, _ => ExpireSolution(false));
                    }
                    else
                    {
                        ExpireSolution(false);
                    }
                }));
            }
            catch (Exception ex)
            {
                LogError(ex);
            }
        }


        private void ResetData()
        {
            while (poses.TryDequeue(out _))
            {
            }

            Interlocked.Exchange(ref poseCount, 0);
            ResetPoseChange();
            SetLatestStatus("[RESET] Cleared cached data.");
            NotifySolution();
        }

        private void UpdatePoseChange(RobotPose pose)
        {
            lock (poseChangeLock)
            {
                if (lastPose == null)
                {
                    lastPose = pose;
                    positionChanged ++;
                    return;
                }

                var dx = Math.Abs(pose.X - lastPose.X);
                var dy = Math.Abs(pose.Y - lastPose.Y);
                var dz = Math.Abs(pose.Z - lastPose.Z);
                var da = Math.Abs(pose.A - lastPose.A);
                var db = Math.Abs(pose.B - lastPose.B);
                var dc = Math.Abs(pose.C - lastPose.C);
                bool change = false;

                if (change = dx > 0.1 || dy > 0.1 || dz > 0.1 || da > 0.1 || db > 0.1 || dc > 0.1)
                { positionChanged++; }

                lastPose = pose;
            }
        }

        private void ResetPoseChange()
        {
            lock (poseChangeLock)
            {
                lastPose = null;
                positionChanged = 0;
            }
        }

        private int GetPositionChanged()
        {
            lock (poseChangeLock)
            {
                return positionChanged;
            }
        }


        private void SetActiveListener(TcpListener listener)
        {
            lock (listenerLock)
            {
                activeListener = listener;
            }
        }

        private void ClearActiveListener(TcpListener listener)
        {
            lock (listenerLock)
            {
                if (ReferenceEquals(activeListener, listener))
                {
                    activeListener = null;
                }
            }
        }

        private void StopActiveListener()
        {
            lock (listenerLock)
            {
                try
                {
                    activeListener?.Stop();
                }
                catch (Exception ex)
                {
                    LogError(ex);
                }
            }
        }

        private void LogServerBanner()
        {
            LogMessage(new string('=', 60));
            LogMessage("  KUKA EKI Flange-Pose TCP Server");
            LogMessage($"  Listening on 0.0.0.0:{Port}");
            LogMessage("  Output: Grasshopper component (format: XML FLANGE)");
            LogMessage(new string('=', 60));
            LogMessage("Waiting for robot connection ...");
        }

        private void LogPose(RobotPose pose)
        {
            var index = Interlocked.Read(ref poseCount);
            LogMessage(
                $"  [{index,4}]  " +
                $"X={pose.X,9:0.00}  Y={pose.Y,9:0.00}  Z={pose.Z,9:0.00}  " +
                $"A={pose.A,8:0.00}  B={pose.B,8:0.00}  C={pose.C,8:0.00}  " +
                $"E1={pose.E1:0.0} E2={pose.E2:0.0} E3={pose.E3:0.0} E4={pose.E4:0.0}");
        }

        private void LogMessage(string message)
        {
            try
            {
                SetLatestStatus(message);
                NotifySolution();
            }
            catch
            {
            }
        }

        private void LogError(Exception ex)
        {
            var message = ex == null ? "Unknown error." : ex.ToString();
            try
            {
                SetLatestStatus(message);
                NotifySolution();
            }
            catch
            {
            }
        }

        private string GetLatestStatus()
        {
            lock (statusLock)
            {
                return latestStatus;
            }
        }

        private void SetLatestStatus(string message)
        {
            lock (statusLock)
            {
                latestStatus = message ?? string.Empty;
            }
        }
    }
}