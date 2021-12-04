//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        ///VARIAVEIS USADAS NA CAPTURA DOS ANGULOS
        double xinicialombrod;
        double yinicialombrod;
        double zinicialombrod;

        double xinicialcotovelod;
        double yinicialcotovelod;
        double zinicialcotovelod;

        double xinicialombroe;
        double yinicialombroe;
        double zinicialombroe;

        double xinicialcotoveloe;
        double yinicialcotoveloe;
        double zinicialcotoveloe;

        double xinicialpunhod;
        double yinicialpunhod;
        double zinicialpunhod;

        double xinicialpunhoe;
        double yinicialpunhoe;
        double zinicialpunhoe;

        double xinicialpolegard;
        double yinicialpolegard;
        double zinicialpolegard;

        double xinicialpolegare;
        double yinicialpolegare;
        double zinicialpolegare;

        double xinicialmaod;
        double yinicialmaod;
        double zinicialmaod;

        double xinicialmaoe;
        double yinicialmaoe;
        double zinicialmaoe;

        double[] flexaoombrod = new double[5];
        double[] flexaoombroe = new double[5];

        double[] abducaoombrod = new double[5];
        double[] abducaoombroe = new double[5];

        double[] aducaoombrod = new double[5];
        double[] aducaoombroe = new double[5];

        double[] flexaocotovelod = new double[5];
        double[] flexaocotoveloe = new double[5];

        double[] pronacaocotovelod = new double[5];
        double[] pronacaocotoveloe = new double[5];

        int flag = 0;
        int flag1 = 0;
        int flag2 = 0;
        int flag3 = 0;
        int flag4 = 0;
        int flag5 = 0;
        int flag6 = 0;
        int flag7 = 0;
        int flag8 = 0;
        int flag9 = 0;
        int flag10 = 0;
        int flag11 = 0;
        int flag12 = 0;
        int flag13 = 0;
        int flag14 = 0;
        int flag15 = 0;
        int flag16 = 0;
        int flag17 = 0;
        int flag18 = 0;
        int flag19 = 0;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
            for (int i = 0; i < 5; i++)
            {
                flexaoombrod[i] = 999999;
                flexaoombroe[i] = 999999;

                abducaoombrod[i] = 999999;
                abducaoombroe[i] = 999999;

                aducaoombrod[i] = 999999;
                aducaoombroe[i] = 999999;

                flexaocotovelod[i] = 999999;
                flexaocotoveloe[i] = 999999;

                pronacaocotovelod[i] = 999999;
                pronacaocotoveloe[i] = 999999;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;


            //CHAMA A FUNCAO DE CAPTURA DOS ANGULOS
            Captura_Angulos(e.FrameReference.AcquireFrame());


            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {

                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }            
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {

                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        private void Captura_Angulos(BodyFrame quadro)
        {
            Body esqueleto = GetActiveBody(quadro);
            if (esqueleto != null)
            {
                double xspine = esqueleto.Joints[JointType.SpineBase].Position.X;
                double yspine = esqueleto.Joints[JointType.SpineBase].Position.Y;
                double zspine = esqueleto.Joints[JointType.SpineBase].Position.Z;

                double xcentro = esqueleto.Joints[JointType.SpineShoulder].Position.X;
                double ycentro = esqueleto.Joints[JointType.SpineShoulder].Position.Y;
                double zcentro = esqueleto.Joints[JointType.SpineShoulder].Position.Z;

                double xombrod = esqueleto.Joints[JointType.ShoulderRight].Position.X;
                double yombrod = esqueleto.Joints[JointType.ShoulderRight].Position.Y;
                double zombrod = esqueleto.Joints[JointType.ShoulderRight].Position.Z;

                double xombroe = esqueleto.Joints[JointType.ShoulderLeft].Position.X;
                double yombroe = esqueleto.Joints[JointType.ShoulderLeft].Position.Y;
                double zombroe = esqueleto.Joints[JointType.ShoulderLeft].Position.Z;

                double xcotovelod = esqueleto.Joints[JointType.ElbowRight].Position.X;
                double ycotovelod = esqueleto.Joints[JointType.ElbowRight].Position.Y;
                double zcotovelod = esqueleto.Joints[JointType.ElbowRight].Position.Z;

                double xcotoveloe = esqueleto.Joints[JointType.ElbowLeft].Position.X;
                double ycotoveloe = esqueleto.Joints[JointType.ElbowLeft].Position.Y;
                double zcotoveloe = esqueleto.Joints[JointType.ElbowLeft].Position.Z;

                double xpulsod = esqueleto.Joints[JointType.WristRight].Position.X;
                double ypulsod = esqueleto.Joints[JointType.WristRight].Position.Y;
                double zpulsod = esqueleto.Joints[JointType.WristRight].Position.Z;

                double xpulsoe = esqueleto.Joints[JointType.WristLeft].Position.X;
                double ypulsoe = esqueleto.Joints[JointType.WristLeft].Position.Y;
                double zpulsoe = esqueleto.Joints[JointType.WristLeft].Position.Z;

                double xmaod = esqueleto.Joints[JointType.HandRight].Position.X;
                double ymaod = esqueleto.Joints[JointType.HandRight].Position.Y;
                double zmaod = esqueleto.Joints[JointType.HandRight].Position.Z;

                double xmaoe = esqueleto.Joints[JointType.HandLeft].Position.X;
                double ymaoe = esqueleto.Joints[JointType.HandLeft].Position.Y;
                double zmaoe = esqueleto.Joints[JointType.HandLeft].Position.Z;

                double xpolegard = esqueleto.Joints[JointType.ThumbRight].Position.X;
                double ypolegard = esqueleto.Joints[JointType.ThumbRight].Position.Y;
                double zpolegard = esqueleto.Joints[JointType.ThumbRight].Position.Z;

                double xpolegare = esqueleto.Joints[JointType.ThumbLeft].Position.X;
                double ypolegare = esqueleto.Joints[JointType.ThumbLeft].Position.Y;
                double zpolegare = esqueleto.Joints[JointType.ThumbLeft].Position.Z;

                //FLEXAO OMBRO D  
                if (flexao_extensao_ombro.IsChecked.HasValue && flexao_extensao_ombro.IsChecked.Value)
                {
                    
                    StreamWriter escreve = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\pontos.txt", true);
                    escreve.WriteLine(xspine + " " + yspine + " " + zspine + " " + xcentro + " " + ycentro + " " + zcentro + " " + xombrod + " " + yombrod + " " + zombrod + " " + xombroe + " " + yombroe + " " + zombroe + " " + xcotovelod + " " + ycotovelod + " " + zcotovelod + " " + xcotoveloe + " " + ycotoveloe + " " + zcotoveloe + " " + xpulsod + " " + ypulsod + " " + zpulsod + " " + xpulsoe + " " + ypulsoe + " " + zpulsoe + " " + xmaod + " " + ymaod + " " + zmaod + " " + xmaoe + " " + ymaoe + " " + zmaoe);
                    escreve.Close();
                    
                    if (flag == 0)
                    {
                        yinicialombrod = yombrod;
                        zinicialombrod = zombrod;
                        yinicialcotovelod = ycotovelod;
                        zinicialcotovelod = zcotovelod;
                        flag = 1;
                    }

                    if ((flexaoombrod[0] != 999999) && (flexaoombrod[1] != 999999) && (flexaoombrod[2] != 999999) && (flexaoombrod[3] != 999999) && (flexaoombrod[4] != 999999))
                    {
                        if (flag1 == 0)
                        {
                            StreamWriter escreve1 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\flexaoombrod.txt", true);
                            escreve1.WriteLine(flexaoombrod[4]);
                            escreve1.WriteLine(flexaoombrod[3]);
                            escreve1.WriteLine(flexaoombrod[2]);
                            escreve1.WriteLine(flexaoombrod[1]);
                            escreve1.WriteLine(flexaoombrod[0]);
                            escreve1.Close();
                            flag1 = 1;
                        }
                        for (int i = 4; i > 0; i--)
                        {
                            flexaoombrod[i] = flexaoombrod[i - 1];
                        }
                        flexaoombrod[0] = 1.04102864 * Flexao_Extensao_Ombro(ycotovelod, zcotovelod, yombrod, zombrod, yinicialcotovelod, zinicialcotovelod, yinicialombrod, zinicialombrod);

                        flexaoombrod[0] = media_movel(flexaoombrod);
                        
                        StreamWriter escreve2 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\flexaoombrod.txt", true);
                        escreve2.WriteLine(flexaoombrod[0]);
                        escreve2.Close();
                        
                        Console.WriteLine(flexaoombrod[0]);
                    }
                    else
                    {
                        for (int i = 4; i > 0; i--)
                        {
                            flexaoombrod[i] = flexaoombrod[i - 1];
                        }
                        flexaoombrod[0] = 1.04102864 * Flexao_Extensao_Ombro(ycotovelod, zcotovelod, yombrod, zombrod, yinicialcotovelod, zinicialcotovelod, yinicialombrod, zinicialombrod);
                    }
                }
                else
                {
                    flag = 0;
                }

                //ABDUCAO OMBRO D
                if (abducao_ombro.IsChecked.HasValue && abducao_ombro.IsChecked.Value)
                {
                    StreamWriter escreve3 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\pontos.txt", true);
                    escreve3.WriteLine(xspine + " " + yspine + " " + zspine + " " + xcentro + " " + ycentro + " " + zcentro + " " + xombrod + " " + yombrod + " " + zombrod + " " + xombroe + " " + yombroe + " " + zombroe + " " + xcotovelod + " " + ycotovelod + " " + zcotovelod + " " + xcotoveloe + " " + ycotoveloe + " " + zcotoveloe + " " + xpulsod + " " + ypulsod + " " + zpulsod + " " + xpulsoe + " " + ypulsoe + " " + zpulsoe + " " + xmaod + " " + ymaod + " " + zmaod + " " + xmaoe + " " + ymaoe + " " + zmaoe);
                    escreve3.Close();

                    if (flag4 == 0)
                    {
                        xinicialombrod = xombrod;
                        yinicialombrod = yombrod;
                        xinicialcotovelod = xcotovelod;
                        yinicialcotovelod = ycotovelod;
                        flag4 = 1;
                    }

                    if ((abducaoombrod[0] != 999999) && (abducaoombrod[1] != 999999) && (abducaoombrod[2] != 999999) && (abducaoombrod[3] != 999999) && (abducaoombrod[4] != 999999))
                    {
                        if (flag5 == 0)
                        {
                            StreamWriter escreve4 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\abducaoombrod.txt", true);
                            escreve4.WriteLine(abducaoombrod[4]);
                            escreve4.WriteLine(abducaoombrod[3]);
                            escreve4.WriteLine(abducaoombrod[2]);
                            escreve4.WriteLine(abducaoombrod[1]);
                            escreve4.WriteLine(abducaoombrod[0]);
                            escreve4.Close();
                            flag5 = 1;
                        }
                        for (int i = 4; i > 0; i--)
                        {
                            abducaoombrod[i] = abducaoombrod[i - 1];
                        }
                        abducaoombrod[0] = 1.0598467 * Abducao_Ombro(xcotovelod, ycotovelod, xombrod, yombrod, xinicialcotovelod, yinicialcotovelod, xinicialombrod, yinicialombrod);

                        abducaoombrod[0] = media_movel(abducaoombrod);

                        StreamWriter escreve5 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\abducaoombrod.txt", true);
                        escreve5.WriteLine(abducaoombrod[0]);
                        escreve5.Close();

                        Console.WriteLine(abducaoombrod[0]);
                    }
                    else
                    {
                        for (int i = 4; i > 0; i--)
                        {
                            abducaoombrod[i] = abducaoombrod[i - 1];
                        }
                        abducaoombrod[0] = 1.0598467 * Abducao_Ombro(xcotovelod, ycotovelod, xombrod, yombrod, xinicialcotovelod, yinicialcotovelod, xinicialombrod, yinicialombrod);
                    }
                }
                else
                {
                    flag4 = 0;
                }

                //ADUCAO OMBRO D
                if (aducao_ombro.IsChecked.HasValue && aducao_ombro.IsChecked.Value)
                {
                    StreamWriter escreve13 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\pontos.txt", true);
                    escreve13.WriteLine(xspine + " " + yspine + " " + zspine + " " + xcentro + " " + ycentro + " " + zcentro + " " + xombrod + " " + yombrod + " " + zombrod + " " + xombroe + " " + yombroe + " " + zombroe + " " + xcotovelod + " " + ycotovelod + " " + zcotovelod + " " + xcotoveloe + " " + ycotoveloe + " " + zcotoveloe + " " + xpulsod + " " + ypulsod + " " + zpulsod + " " + xpulsoe + " " + ypulsoe + " " + zpulsoe + " " + xmaod + " " + ymaod + " " + zmaod + " " + xmaoe + " " + ymaoe + " " + zmaoe);
                    escreve13.Close();

                    if (flag8 == 0)
                    {
                        xinicialombrod = xombrod;
                        zinicialombrod = zombrod;
                        xinicialcotovelod = xcotovelod;
                        zinicialcotovelod = zcotovelod;
                        flag8 = 1;
                    }

                    if ((aducaoombrod[0] != 999999) && (aducaoombrod[1] != 999999) && (aducaoombrod[2] != 999999) && (aducaoombrod[3] != 999999) && (aducaoombrod[4] != 999999))
                    {
                        if (flag9 == 0)
                        {
                            StreamWriter escreve14 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\aducaoombrod.txt", true);
                            escreve14.WriteLine(aducaoombrod[4]);
                            escreve14.WriteLine(aducaoombrod[3]);
                            escreve14.WriteLine(aducaoombrod[2]);
                            escreve14.WriteLine(aducaoombrod[1]);
                            escreve14.WriteLine(aducaoombrod[0]);
                            escreve14.Close();
                            flag9 = 1;
                        }
                        for (int i = 4; i > 0; i--)
                        {
                            aducaoombrod[i] = aducaoombrod[i - 1];
                        }
                        aducaoombrod[0] = 0.786422977 * Aducao_Ombro(xcotovelod, zcotovelod, xombrod, zombrod, xinicialcotovelod, zinicialcotovelod, xinicialombrod, zinicialombrod);

                        aducaoombrod[0] = media_movel(aducaoombrod);

                        StreamWriter escreve15 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\aducaoombrod.txt", true);
                        escreve15.WriteLine(aducaoombrod[0]);
                        escreve15.Close();

                        Console.WriteLine(aducaoombrod[0]);
                    }
                    else
                    {
                        for (int i = 4; i > 0; i--)
                        {
                            aducaoombrod[i] = aducaoombrod[i - 1];
                        }
                        aducaoombrod[0] = 0.786422977 * Aducao_Ombro(xcotovelod, zcotovelod, xombrod, zombrod, xinicialcotovelod, zinicialcotovelod, xinicialombrod, zinicialombrod);
                    }
                }
                else
                {
                    flag8 = 0;
                }

                //FLEXAO OMBRO E
                if (flexao_extensao_ombro_esquerdo.IsChecked.HasValue && flexao_extensao_ombro_esquerdo.IsChecked.Value)
                {
                    StreamWriter escreve7 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\pontos.txt", true);
                    escreve7.WriteLine(xspine + " " + yspine + " " + zspine + " " + xcentro + " " + ycentro + " " + zcentro + " " + xombrod + " " + yombrod + " " + zombrod + " " + xombroe + " " + yombroe + " " + zombroe + " " + xcotovelod + " " + ycotovelod + " " + zcotovelod + " " + xcotoveloe + " " + ycotoveloe + " " + zcotoveloe + " " + xpulsod + " " + ypulsod + " " + zpulsod + " " + xpulsoe + " " + ypulsoe + " " + zpulsoe + " " + xmaod + " " + ymaod + " " + zmaod + " " + xmaoe + " " + ymaoe + " " + zmaoe);
                    escreve7.Close();

                    if (flag2 == 0)
                    {
                        yinicialombroe = yombroe;
                        zinicialombroe = zombroe;
                        yinicialcotoveloe = ycotoveloe;
                        zinicialcotoveloe = zcotoveloe;
                        flag2 = 1;
                    }

                    if ((flexaoombroe[0] != 999999) && (flexaoombroe[1] != 999999) && (flexaoombroe[2] != 999999) && (flexaoombroe[3] != 999999) && (flexaoombroe[4] != 999999))
                    {
                        if (flag3 == 0)
                        {
                            StreamWriter escreve8 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\flexaoombroe.txt", true);
                            escreve8.WriteLine(flexaoombrod[4]);
                            escreve8.WriteLine(flexaoombrod[3]);
                            escreve8.WriteLine(flexaoombrod[2]);
                            escreve8.WriteLine(flexaoombrod[1]);
                            escreve8.WriteLine(flexaoombrod[0]);
                            escreve8.Close();
                            flag3 = 1;
                        }
                        for (int i = 4; i > 0; i--)
                        {
                            flexaoombroe[i] = flexaoombroe[i - 1];
                        }
                        flexaoombroe[0] = 1.04102864 * Flexao_Extensao_Ombro(ycotoveloe, zcotoveloe, yombroe, zombroe, yinicialcotoveloe, zinicialcotoveloe, yinicialombroe, zinicialombroe);

                        flexaoombroe[0] = media_movel(flexaoombroe);

                        StreamWriter escreve9 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\flexaoombroe.txt", true);
                        escreve9.WriteLine(flexaoombroe[0]);
                        escreve9.Close();

                        Console.WriteLine(flexaoombroe[0]);
                    }
                    else
                    {
                        for (int i = 4; i > 0; i--)
                        {
                            flexaoombroe[i] = flexaoombroe[i - 1];
                        }
                        flexaoombroe[0] = 1.04102864 * Flexao_Extensao_Ombro(ycotoveloe, zcotoveloe, yombroe, zombroe, yinicialcotoveloe, zinicialcotoveloe, yinicialombroe, zinicialombroe);
                    }
                }
                else
                {
                    flag2 = 0;
                }

                //ABDUCAO OMBRO E
                if (abducao_ombro_esquerdo.IsChecked.HasValue && abducao_ombro_esquerdo.IsChecked.Value)
                {
                    StreamWriter escreve10 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\pontos.txt", true);
                    escreve10.WriteLine(xspine + " " + yspine + " " + zspine + " " + xcentro + " " + ycentro + " " + zcentro + " " + xombrod + " " + yombrod + " " + zombrod + " " + xombroe + " " + yombroe + " " + zombroe + " " + xcotovelod + " " + ycotovelod + " " + zcotovelod + " " + xcotoveloe + " " + ycotoveloe + " " + zcotoveloe + " " + xpulsod + " " + ypulsod + " " + zpulsod + " " + xpulsoe + " " + ypulsoe + " " + zpulsoe + " " + xmaod + " " + ymaod + " " + zmaod + " " + xmaoe + " " + ymaoe + " " + zmaoe);
                    escreve10.Close();

                    if (flag6 == 0)
                    {
                        xinicialombroe = xombroe;
                        yinicialombroe = yombroe;
                        xinicialcotoveloe = xcotoveloe;
                        yinicialcotoveloe = ycotoveloe;
                        flag6 = 1;
                    }

                    if ((abducaoombroe[0] != 999999) && (abducaoombroe[1] != 999999) && (abducaoombroe[2] != 999999) && (abducaoombroe[3] != 999999) && (abducaoombroe[4] != 999999))
                    {
                        if (flag7 == 0)
                        {
                            StreamWriter escreve11 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\abducaoombroe.txt", true);
                            escreve11.WriteLine(abducaoombroe[4]);
                            escreve11.WriteLine(abducaoombroe[3]);
                            escreve11.WriteLine(abducaoombroe[2]);
                            escreve11.WriteLine(abducaoombroe[1]);
                            escreve11.WriteLine(abducaoombroe[0]);
                            escreve11.Close();
                            flag7 = 1;
                        }
                        for (int i = 4; i > 0; i--)
                        {
                            abducaoombroe[i] = abducaoombroe[i - 1];
                        }
                        abducaoombroe[0] = 1.0598467 * Abducao_Ombro(xcotoveloe, ycotoveloe, xombroe, yombroe, xinicialcotoveloe, yinicialcotoveloe, xinicialombroe, yinicialombroe);

                        abducaoombroe[0] = media_movel(abducaoombroe);

                        StreamWriter escreve12 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\abducaoombroe.txt", true);
                        escreve12.WriteLine(abducaoombroe[0]);
                        escreve12.Close();

                        Console.WriteLine(abducaoombroe[0]);
                    }
                    else
                    {
                        for (int i = 4; i > 0; i--)
                        {
                            abducaoombroe[i] = abducaoombroe[i - 1];
                        }
                        abducaoombroe[0] = 1.0598467 * Abducao_Ombro(xcotoveloe, ycotoveloe, xombroe, yombroe, xinicialcotoveloe, yinicialcotoveloe, xinicialombroe, yinicialombroe);
                    }
                }
                else
                {
                    flag6 = 0;
                }

                //ADUCAO OMBRO E
                if (aducao_ombro_esquerdo.IsChecked.HasValue && aducao_ombro_esquerdo.IsChecked.Value)
                {
                    StreamWriter escreve16 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\pontos.txt", true);
                    escreve16.WriteLine(xspine + " " + yspine + " " + zspine + " " + xcentro + " " + ycentro + " " + zcentro + " " + xombrod + " " + yombrod + " " + zombrod + " " + xombroe + " " + yombroe + " " + zombroe + " " + xcotovelod + " " + ycotovelod + " " + zcotovelod + " " + xcotoveloe + " " + ycotoveloe + " " + zcotoveloe + " " + xpulsod + " " + ypulsod + " " + zpulsod + " " + xpulsoe + " " + ypulsoe + " " + zpulsoe + " " + xmaod + " " + ymaod + " " + zmaod + " " + xmaoe + " " + ymaoe + " " + zmaoe);
                    escreve16.Close();

                    if (flag10 == 0)
                    {
                        xinicialombroe = xombroe;
                        zinicialombroe = zombroe;
                        xinicialcotoveloe = xcotoveloe;
                        zinicialcotoveloe = zcotoveloe;
                        flag10 = 1;
                    }

                    if ((aducaoombroe[0] != 999999) && (aducaoombroe[1] != 999999) && (aducaoombroe[2] != 999999) && (aducaoombroe[3] != 999999) && (aducaoombroe[4] != 999999))
                    {
                        if (flag11 == 0)
                        {
                            StreamWriter escreve17 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\aducaoombroe.txt", true);
                            escreve17.WriteLine(aducaoombroe[4]);
                            escreve17.WriteLine(aducaoombroe[3]);
                            escreve17.WriteLine(aducaoombroe[2]);
                            escreve17.WriteLine(aducaoombroe[1]);
                            escreve17.WriteLine(aducaoombroe[0]);
                            escreve17.Close();
                            flag11 = 1;
                        }
                        for (int i = 4; i > 0; i--)
                        {
                            aducaoombroe[i] = aducaoombroe[i - 1];
                        }
                        aducaoombroe[0] = 0.786422977 * Aducao_Ombro(xcotoveloe, zcotoveloe, xombroe, zombroe, xinicialcotoveloe, zinicialcotoveloe, xinicialombroe, zinicialombroe);

                        aducaoombroe[0] = media_movel(aducaoombroe);

                        StreamWriter escreve18 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\aducaoombroe.txt", true);
                        escreve18.WriteLine(aducaoombroe[0]);
                        escreve18.Close();

                        Console.WriteLine(aducaoombroe[0]);
                    }
                    else
                    {
                        for (int i = 4; i > 0; i--)
                        {
                            aducaoombroe[i] = aducaoombroe[i - 1];
                        }
                        aducaoombroe[0] = 0.786422977 * Aducao_Ombro(xcotoveloe, zcotoveloe, xombroe, zombroe, xinicialcotoveloe, zinicialcotoveloe, xinicialombroe, zinicialombroe);
                    }
                }
                else
                {
                    flag10 = 0;
                }

                //FLEXAO E EXTENSAO COTOVELO D
                if (flexao_extensao_cotovelo_d.IsChecked.HasValue && flexao_extensao_cotovelo_d.IsChecked.Value)
                {
                     StreamWriter escreve19 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\pontos.txt", true);
                     escreve19.WriteLine(xspine + " " + yspine + " " + zspine + " " + xcentro + " " + ycentro + " " + zcentro + " " + xombrod + " " + yombrod + " " + zombrod + " " + xombroe + " " + yombroe + " " + zombroe + " " + xcotovelod + " " + ycotovelod + " " + zcotovelod + " " + xcotoveloe + " " + ycotoveloe + " " + zcotoveloe + " " + xpulsod + " " + ypulsod + " " + zpulsod + " " + xpulsoe + " " + ypulsoe + " " + zpulsoe + " " + xmaod + " " + ymaod + " " + zmaod + " " + xmaoe + " " + ymaoe + " " + zmaoe + " " + xpolegard + " " + ypolegard + " " + zpolegard + " " + xpolegare + " " + ypolegare + " " + zpolegare);
                     escreve19.Close();

                    if (flag12 == 0)
                    {
                        xinicialcotovelod = xcotovelod;
                        yinicialcotovelod = ycotovelod;
                        xinicialpunhod = xpulsod;
                        yinicialpunhod = ypulsod;
                        flag12 = 1;
                    }

                    if ((flexaocotovelod[0] != 999999) && (flexaocotovelod[1] != 999999) && (flexaocotovelod[2] != 999999) && (flexaocotovelod[3] != 999999) && (flexaocotovelod[4] != 999999))
                    {
                        if (flag13 == 0)
                        {
                            StreamWriter escreve20 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\flexaocotovelod.txt", true);
                            escreve20.WriteLine(flexaocotovelod[4]);
                            escreve20.WriteLine(flexaocotovelod[3]);
                            escreve20.WriteLine(flexaocotovelod[2]);
                            escreve20.WriteLine(flexaocotovelod[1]);
                            escreve20.WriteLine(flexaocotovelod[0]);
                            escreve20.Close();
                            flag13 = 1;
                        }
                        for (int i = 4; i > 0; i--)
                        {
                            flexaocotovelod[i] = flexaocotovelod[i - 1];
                        }
                        flexaocotovelod[0] = 0.841110455 * Flexao_Extensao_Ombro(xpulsod, ypulsod, xcotovelod, ycotovelod, xinicialpunhod, yinicialpunhod, xinicialcotovelod, yinicialcotovelod);

                        flexaocotovelod[0] = media_movel(flexaocotovelod);

                        StreamWriter escreve21 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\flexaocotovelod.txt", true);
                        escreve21.WriteLine(flexaocotovelod[0]);
                        escreve21.Close();

                        Console.WriteLine(flexaocotovelod[0]);
                    }
                    else
                    {
                        for (int i = 4; i > 0; i--)
                        {
                            flexaocotovelod[i] = flexaocotovelod[i - 1];
                        }
                        flexaocotovelod[0] = 0.841110455 * Flexao_Extensao_Ombro(xpulsod, ypulsod, xcotovelod, ycotovelod, xinicialpunhod, yinicialpunhod, xinicialcotovelod, yinicialcotovelod);
                    }
                }
                else
                {
                    flag12 = 0;
                }

                //PRONAÇÃO E SUPINAÇÃO COTOVELO D
                if (pronacao_supinacao_cotovelo_d.IsChecked.HasValue && pronacao_supinacao_cotovelo_d.IsChecked.Value)
                {
                    StreamWriter escreve22 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\pontos.txt", true);
                    escreve22.WriteLine(xspine + " " + yspine + " " + zspine + " " + xcentro + " " + ycentro + " " + zcentro + " " + xombrod + " " + yombrod + " " + zombrod + " " + xombroe + " " + yombroe + " " + zombroe + " " + xcotovelod + " " + ycotovelod + " " + zcotovelod + " " + xcotoveloe + " " + ycotoveloe + " " + zcotoveloe + " " + xpulsod + " " + ypulsod + " " + zpulsod + " " + xpulsoe + " " + ypulsoe + " " + zpulsoe + " " + xmaod + " " + ymaod + " " + zmaod + " " + xmaoe + " " + ymaoe + " " + zmaoe + " " + xpolegard + " " + ypolegard + " " + zpolegard + " " + xpolegare + " " + ypolegare + " " + zpolegare);
                    escreve22.Close();

                    if (flag14 == 0)
                    {
                        xinicialpolegard = xpolegard;
                        yinicialpolegard = ypolegard;
                        xinicialmaod = xmaod;
                        yinicialmaod = ymaod;
                        flag14 = 1;
                    }

                    if ((pronacaocotovelod[0] != 999999) && (pronacaocotovelod[1] != 999999) && (pronacaocotovelod[2] != 999999) && (pronacaocotovelod[3] != 999999) && (pronacaocotovelod[4] != 999999))
                    {
                        if (flag15 == 0)
                        {
                            StreamWriter escreve23 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\pronacaocotovelod.txt", true);
                            escreve23.WriteLine(pronacaocotovelod[4]);
                            escreve23.WriteLine(pronacaocotovelod[3]);
                            escreve23.WriteLine(pronacaocotovelod[2]);
                            escreve23.WriteLine(pronacaocotovelod[1]);
                            escreve23.WriteLine(pronacaocotovelod[0]);
                            escreve23.Close();
                            flag15 = 1;
                        }
                        for (int i = 4; i > 0; i--)
                        {
                            pronacaocotovelod[i] = pronacaocotovelod[i - 1];
                        }
                        pronacaocotovelod[0] = 0.990987654 * Abducao_Ombro(xmaod, ymaod, xpolegard, ypolegard, xinicialmaod, yinicialmaod, xinicialpolegard, yinicialpolegard);

                        pronacaocotovelod[0] = media_movel(pronacaocotovelod);

                        StreamWriter escreve24 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\pronacaocotovelod.txt", true);
                        escreve24.WriteLine(pronacaocotovelod[0]);
                        escreve24.Close();

                        Console.WriteLine(pronacaocotovelod[0]);
                    }
                    else
                    {
                        for (int i = 4; i > 0; i--)
                        {
                            pronacaocotovelod[i] = pronacaocotovelod[i - 1];
                        }
                        pronacaocotovelod[0] = 0.990987654 * Abducao_Ombro(xmaod, ymaod, xpolegard, ypolegard, xinicialmaod, yinicialmaod, xinicialpolegard, yinicialpolegard);
                    }
                }
                else
                {
                    flag14 = 0;
                }


                //FLEXAO E EXTENSAO COTOVELO E
                if (flexao_extensao_cotovelo_e.IsChecked.HasValue && flexao_extensao_cotovelo_e.IsChecked.Value)
                {
                    StreamWriter escreve25 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\pontos.txt", true);
                    escreve25.WriteLine(xspine + " " + yspine + " " + zspine + " " + xcentro + " " + ycentro + " " + zcentro + " " + xombrod + " " + yombrod + " " + zombrod + " " + xombroe + " " + yombroe + " " + zombroe + " " + xcotovelod + " " + ycotovelod + " " + zcotovelod + " " + xcotoveloe + " " + ycotoveloe + " " + zcotoveloe + " " + xpulsod + " " + ypulsod + " " + zpulsod + " " + xpulsoe + " " + ypulsoe + " " + zpulsoe + " " + xmaod + " " + ymaod + " " + zmaod + " " + xmaoe + " " + ymaoe + " " + zmaoe + " " + xpolegard + " " + ypolegard + " " + zpolegard + " " + xpolegare + " " + ypolegare + " " + zpolegare);
                    escreve25.Close();

                    if (flag16 == 0)
                    {
                        xinicialcotoveloe = xcotoveloe;
                        yinicialcotoveloe = ycotoveloe;
                        xinicialpunhoe = xpulsoe;
                        yinicialpunhoe = ypulsoe;
                        flag16 = 1;
                    }

                    if ((flexaocotoveloe[0] != 999999) && (flexaocotoveloe[1] != 999999) && (flexaocotoveloe[2] != 999999) && (flexaocotoveloe[3] != 999999) && (flexaocotoveloe[4] != 999999))
                    {
                        if (flag17 == 0)
                        {
                            StreamWriter escreve26 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\flexaocotoveloe.txt", true);
                            escreve26.WriteLine(flexaocotoveloe[4]);
                            escreve26.WriteLine(flexaocotoveloe[3]);
                            escreve26.WriteLine(flexaocotoveloe[2]);
                            escreve26.WriteLine(flexaocotoveloe[1]);
                            escreve26.WriteLine(flexaocotoveloe[0]);
                            escreve26.Close();
                            flag17 = 1;
                        }
                        for (int i = 4; i > 0; i--)
                        {
                            flexaocotoveloe[i] = flexaocotoveloe[i - 1];
                        }
                        flexaocotoveloe[0] = 0.841110455 * Flexao_Extensao_Ombro(xpulsoe, ypulsoe, xcotoveloe, ycotoveloe, xinicialpunhoe, yinicialpunhoe, xinicialcotoveloe, yinicialcotoveloe);

                        flexaocotoveloe[0] = media_movel(flexaocotoveloe);

                        StreamWriter escreve27 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\flexaocotoveloe.txt", true);
                        escreve27.WriteLine(flexaocotoveloe[0]);
                        escreve27.Close();

                        Console.WriteLine(flexaocotoveloe[0]);
                    }
                    else
                    {
                        for (int i = 4; i > 0; i--)
                        {
                            flexaocotoveloe[i] = flexaocotoveloe[i - 1];
                        }
                        flexaocotoveloe[0] = 0.841110455 * Flexao_Extensao_Ombro(xpulsoe, ypulsoe, xcotoveloe, ycotoveloe, xinicialpunhoe, yinicialpunhoe, xinicialcotoveloe, yinicialcotoveloe);
                    }
                }
                else
                {
                    flag16 = 0;
                }

                //PRONAÇÃO E SUPINAÇÃO COTOVELO E
                if (pronacao_supinacao_cotovelo_e.IsChecked.HasValue && pronacao_supinacao_cotovelo_e.IsChecked.Value)
                {
                    StreamWriter escreve28 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\pontos.txt", true);
                    escreve28.WriteLine(xspine + " " + yspine + " " + zspine + " " + xcentro + " " + ycentro + " " + zcentro + " " + xombrod + " " + yombrod + " " + zombrod + " " + xombroe + " " + yombroe + " " + zombroe + " " + xcotovelod + " " + ycotovelod + " " + zcotovelod + " " + xcotoveloe + " " + ycotoveloe + " " + zcotoveloe + " " + xpulsod + " " + ypulsod + " " + zpulsod + " " + xpulsoe + " " + ypulsoe + " " + zpulsoe + " " + xmaod + " " + ymaod + " " + zmaod + " " + xmaoe + " " + ymaoe + " " + zmaoe + " " + xpolegard + " " + ypolegard + " " + zpolegard + " " + xpolegare + " " + ypolegare + " " + zpolegare);
                    escreve28.Close();

                    if (flag18 == 0)
                    {
                        xinicialpolegare = xpolegare;
                        yinicialpolegare = ypolegare;
                        xinicialmaoe = xmaoe;
                        yinicialmaoe = ymaoe;
                        flag18 = 1;
                    }

                    if ((pronacaocotoveloe[0] != 999999) && (pronacaocotoveloe[1] != 999999) && (pronacaocotoveloe[2] != 999999) && (pronacaocotoveloe[3] != 999999) && (pronacaocotoveloe[4] != 999999))
                    {
                        if (flag19 == 0)
                        {
                            StreamWriter escreve29 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\pronacaocotoveloe.txt", true);
                            escreve29.WriteLine(pronacaocotoveloe[4]);
                            escreve29.WriteLine(pronacaocotoveloe[3]);
                            escreve29.WriteLine(pronacaocotoveloe[2]);
                            escreve29.WriteLine(pronacaocotoveloe[1]);
                            escreve29.WriteLine(pronacaocotoveloe[0]);
                            escreve29.Close();
                            flag19 = 1;
                        }
                        for (int i = 4; i > 0; i--)
                        {
                            pronacaocotoveloe[i] = pronacaocotoveloe[i - 1];
                        }
                        pronacaocotoveloe[0] = 0.990987654 * Abducao_Ombro(xmaoe, ymaoe, xpolegare, ypolegare, xinicialmaoe, yinicialmaoe, xinicialpolegare, yinicialpolegare);

                        pronacaocotoveloe[0] = media_movel(pronacaocotoveloe);

                        StreamWriter escreve30 = new StreamWriter("C:\\Users\\Leonardo\\OneDrive\\Área de Trabalho\\BodyBasics-WPF\\pronacaocotoveloe.txt", true);
                        escreve30.WriteLine(pronacaocotoveloe[0]);
                        escreve30.Close();

                        Console.WriteLine(pronacaocotoveloe[0]);
                    }
                    else
                    {
                        for (int i = 4; i > 0; i--)
                        {
                            pronacaocotoveloe[i] = pronacaocotoveloe[i - 1];
                        }
                        pronacaocotoveloe[0] = 0.990987654 * Abducao_Ombro(xmaoe, ymaoe, xpolegare, ypolegare, xinicialmaoe, yinicialmaoe, xinicialpolegare, yinicialpolegare);
                    }
                }
                else
                {
                    flag18 = 0;
                }


            }
        }

        private double Flexao_Extensao_Ombro(double ycotovelo, double zcotovelo, double yombro, double zombro, double yinicialcotovelo, double zinicialcotovelo, double yinicialombro, double zinicialombro)
        {
            double Angulox = 0;

            double shrhY = yinicialcotovelo - yinicialombro;
            double shrhZ = zinicialcotovelo - zinicialombro;

            double unrhY = ycotovelo - yombro;
            double unrhZ = zcotovelo - zombro;

            double hs = vectorNorm(shrhY, shrhZ);
            double hu = vectorNorm(unrhY, unrhZ);
            double mhshu = shrhY * unrhY + shrhZ * unrhZ;

            double x = mhshu / (hu * hs);

            if (x != Double.NaN)
            {
                if (-1 <= x && x <= 1)
                {
                    double angleRadx = Math.Acos(x);
                    Angulox = angleRadx * (180.0 / Math.PI);
                }
            }

            return Angulox;
        }

        private double Abducao_Ombro(double xcotovelo, double ycotovelo, double xombro, double yombro, double xinicialcotovelo, double yinicialcotovelo, double xinicialombro, double yinicialombro)
        {
            double Anguloz = 0;

            double shrhX = xinicialcotovelo - xinicialombro;
            double shrhY = yinicialcotovelo - yinicialombro;

            double unrhX = xcotovelo - xombro;
            double unrhY = ycotovelo - yombro;

            double hs = vectorNorm(shrhX, shrhY);
            double hu = vectorNorm(unrhX, unrhY);
            double mhshu = shrhX * unrhX + shrhY * unrhY;

            double z = mhshu / (hu * hs);

            if (z != Double.NaN)
            {
                if (-1 <= z && z <= 1)
                {
                    double angleRadz = Math.Acos(z);
                    Anguloz = angleRadz * (180.0 / Math.PI);
                }
            }

            return Anguloz;
        }

        private double Aducao_Ombro(double xcotovelo, double zcotovelo, double xombro, double zombro, double xinicialcotovelo, double zinicialcotovelo, double xinicialombro, double zinicialombro)
        {
            double Anguloy = 0;

            double shrhX = xinicialcotovelo - xinicialombro;
            double shrhZ = zinicialcotovelo - zinicialombro;

            double unrhX = xcotovelo - xombro;
            double unrhZ = zcotovelo - zombro;

            double hs = vectorNorm(shrhX, shrhZ);
            double hu = vectorNorm(unrhX, unrhZ);
            double mhshu = shrhX * unrhX + shrhZ * unrhZ;

            double y = mhshu / (hu * hs);

            if (y != Double.NaN)
            {
                if (-1 <= y && y <= 1)
                {
                    double angleRady = Math.Acos(y);
                    Anguloy = angleRady * (180.0 / Math.PI);
                }
            }

            return Anguloy;
        }

        private double vectorNorm(double pt1, double pt2)
        {
            return Math.Sqrt(Math.Pow(pt1, 2) + Math.Pow(pt2, 2));
        }

        private double media_movel(double[] vet1)
        {
            return ((vet1[0] + vet1[1] + vet1[2] + vet1[3] + vet1[4]) / 5);
        }

        private ulong currTrackingId = 0;
        private Body GetActiveBody(BodyFrame quadro)
        {       
            bool dataReceived = false;
            Body saidaex = null;

            try
            {
                using (BodyFrame bodyFrame = quadro)
                {

                    if (bodyFrame != null)
                    {
                        if (this.bodies == null)
                        {
                            this.bodies = new Body[bodyFrame.BodyCount];
                        }

                        bodyFrame.GetAndRefreshBodyData(this.bodies);
                        dataReceived = true;
                    }
                }

                if (dataReceived)
                {
                    if (currTrackingId <= 0)
                    {
                        foreach (Body body in bodies)
                        {
                            if (body.IsTracked)
                            {
                                currTrackingId = body.TrackingId;
                                return body;
                            }
                        }
                        return null;
                    }
                    else
                    {
                        foreach (Body body in this.bodies)
                        {
                            if (body.IsTracked && body.TrackingId == currTrackingId)
                            {
                                return body;
                            }
                        }
                    }

                    currTrackingId = 0;
                    return null;
                }
                return null;
            }
            catch (Exception ex)
            {
                Console.WriteLine("Problema!!");
                Console.WriteLine(ex.Message);

                return saidaex;
            }
        }
        
    }
}
