using System;
using Xamarin.Forms;

namespace Pendulum
{
    public partial class MainPage : ContentPage
    {
        double theta0 = 0;  // Initial position of the Pendulum [deg]
        double dt = 0.1;    // Time difference [s]
        double g = 9.8;     // Acceleration of gravity [m/s^2]

        // PID Parameters tuned by Z-N for its balancing
        double Ku1 = 2.2;   // Ultimate Gain
        double Pu1 = 0.4;   // Oscillation Period [s] at Kp=Ku, Kd=Ki=0;
        double Idtheta = 0; // Error in the Integral term

        // PID Parameters tuned by Z-N for its horizontally moving
        double Ku2 = 0.03;  // Ultimate Gain
        //double Pu2 = 10.7; // Oscillation Period [s] at Kp=Ku, Kd=Ki=0;
        double Pu2 = 17.0;  // Enlarged the parameter value experimentally
        double Idx = 0;     // Error in the Integral term

        double SpaceWidth, SpaceHeight;     // PhysicalSpace (Display Area) size [pixel]

        // Identifiers for the Pendulum control mode
        string control = "StandBy", control0 = "";

        int n = 1;
        bool slowdowned = false;    // enough slowed down or not
        double X0 = 0.0;    // Initial Position of the Fulcrum


        class Pendulum
        {
            public double L = 1.0;          // The physical length of the Pendulum [m]
            public double k = 0.5;          // Damping coefficient
            public double theta { get; set; }  // The angle of the Pendulum [rad]
            public double omega { get; set; }  // The angular velocity of the pendulum [rad/s]
        }
        Pendulum CurrentPendulum = new Pendulum();

        class Fulcrum
        {
            public double x { get; set; }   // Fulcrum position [m]
            public double v { get; set; }   // Fulcrum velocity [m/s]
            public double a { get; set; }   // Fulcrum acceleration [m/s^2]
        }
        Fulcrum CurrentFulcrum = new Fulcrum();


        // Draw the Pendulum System using BoxViews and Label
        BoxView SimplePendulum = new BoxView(){
            BackgroundColor = Color.Blue,
            WidthRequest = 2
        };
        BoxView MovingBelt = new BoxView(){
            BackgroundColor = Color.Black,
            HeightRequest = 1
        };
        BoxView XCenter = new BoxView(){
            BackgroundColor = Color.Black,
            WidthRequest = 1,
            HeightRequest = 10
        };
        BoxView SliderScale = new BoxView(){
            BackgroundColor = Color.Red,
            WidthRequest = 1,
            HeightRequest = 10
        };
        Label SliderScaleLabel = new Label();



        public MainPage()
        {
            InitializeComponent();

            ControlPendulum();
        }


        // Get PhysicalSpace size and Initialize some variables
        void GetSpaceSize(object sender, EventArgs e)
        {
            SpaceWidth = PhysicalSpace.Width;
            SpaceHeight = PhysicalSpace.Height;

            // Display Moving Belt hanging the Pendulum
            MovingBelt.TranslationY = SpaceHeight / 2;
            MovingBelt.WidthRequest = SpaceWidth;
            PhysicalSpace.Children.Add(MovingBelt);

            // Display Slider Scale and Label on the Moving Belt
            SliderScale.TranslationX = SpaceWidth / 2;
            SliderScale.TranslationY = SpaceHeight / 2;
            PhysicalSpace.Children.Add(SliderScale);
            SliderScaleLabel.Text = "0";
            SliderScaleLabel.TranslationX = SpaceWidth / 2 - 10;
            SliderScaleLabel.TranslationY = SpaceHeight / 2 + 10;
            PhysicalSpace.Children.Add(SliderScaleLabel);

            // Set Max and Min of FulcrumSlider
            FulcrumSlider.Minimum = -SpaceWidth / 2;
            FulcrumSlider.Maximum =  SpaceWidth / 2;

            // Set a small scale at the center of the Moving Belt
            XCenter.TranslationX = SpaceWidth / 2;
            XCenter.TranslationY = SpaceHeight / 2;
            PhysicalSpace.Children.Add(XCenter);

            // Display the initial Pendulum
            SimplePendulum.HeightRequest = CurrentPendulum.L * 100;
            PhysicalSpace.Children.Add(SimplePendulum);
            CurrentPendulum.theta = theta0 / 180 * Math.PI;
            DisplayPendulum(0, CurrentPendulum.theta);
        }


        // Display the swinging Pendulum
        void DisplayPendulum(double x, double theta)
        {
            // Transform Physical space to Screen Space
            x*=100; x+=SpaceWidth / 2;

            // When going out of the screen, Display the pendulum at the oppsite side
            if (x > 0) x = x % SpaceWidth;
            if (x < 0) x = SpaceWidth + (x % SpaceWidth);

            // Calculate the Pendulum altitude
            double Lpixel = SimplePendulum.HeightRequest;
            SimplePendulum.TranslationX = x + Lpixel / 2 * Math.Sin(theta);
            SimplePendulum.TranslationY = SpaceHeight / 2 - Lpixel / 2 * (1 - Math.Cos(theta));
            SimplePendulum.Rotation = -theta * 180 / Math.PI;

            // Update the Slider Scale position
            SliderScale.TranslationX = SpaceWidth / 2 + FulcrumSlider.Value;
            SliderScaleLabel.Text = string.Format("{0:0.0}", FulcrumSlider.Value/10);
            SliderScaleLabel.TranslationX = SpaceWidth / 2 + FulcrumSlider.Value - 10;

            // Display some values at the bottom of the screen
            Label1.Text = string.Format(
                "Fulcrum: x={0:0.0}, v={1:0.00}, a={2:0.0}",
                CurrentFulcrum.x, CurrentFulcrum.v, CurrentFulcrum.a);
            Label2.Text = string.Format(
                "Pendulum: theta={0:0.00}, omega={1:0.00}",
                CurrentPendulum.theta, CurrentPendulum.omega);
            Label3.Text = string.Format(
                "Integral Error: Idtheta={0:0.00}, Idx={1:0.00}",
                Idtheta, Idx);
        }


        // Control the Pendulum
        void ControlPendulum()
        {
            double t = 0;
            double dtheta = 0;

            Device.StartTimer(TimeSpan.FromMilliseconds(100), () =>
            {
                bool NextTimerStart = true;
                
                switch (control)
                {
                    case "StandBy":
                        CurrentFulcrum = SlideFulcrum();
                        X0 = CurrentFulcrum.x;
                        break;

                    case "Stop":
                        X0 = CurrentFulcrum.x;
                        CurrentFulcrum.v = 0;
                        CurrentFulcrum.a = 0;                        
                        FulcrumSlider.Value = X0 * 10.0;
                        t = 0;
                        control = "StandBy";
                        break;

                    case "Pause":
                        return NextTimerStart;

                    case "Reset":   // Initialize all values
                        ResetPendulum();
                        control = "StandBy";
                        break;

                    case "SwingUp": // Oscille and SwingUp Fulcrum position
                        CurrentFulcrum = OscillateFulcrum(t);
                        t += dt;    // Increment time
                        if (Math.Abs(CurrentPendulum.theta) > 3.0) control = "Balance";
                        break;

                    case "Balance": // Balance Control Pendulum with PID-Control
                        dtheta = SlowDownPendulum();
                        CurrentFulcrum = BalancePendulum(dtheta);
                        if(slowdowned)  control = "Move";
                        if (Math.Abs(CurrentPendulum.theta) < 1.5) control = "Retry";
                        break;

                    case "Move":    // Horizontally move the Pendulum
                        dtheta = MovePendulum( FulcrumSlider.Value / 10 );
                        CurrentFulcrum = BalancePendulum(dtheta);
                        if (Math.Abs(CurrentPendulum.theta) < 1.5){ // When fail balancing
                            Idtheta = 0; Idx = 0;
                            CurrentPendulum.k = 30.0;   // Slowdown the Pendulum rotation
                            control = "Retry";
                        }
                        break;

                    case "Retry":
                        CurrentFulcrum = ReturnPendulum();
                        if (Math.Abs(CurrentPendulum.omega) < 2.0){
                            CurrentPendulum.k = 0.5;    // Return the Damping Coefficient
                            Idtheta = 0; Idx = 0;
                            slowdowned = false;
                            n = 1;
                            t = 0;
                            X0 = CurrentFulcrum.x;
                            control = "SwingUp";
                        }
                        break;
                    
                }
                StatusLabel.Text = "Status: " + control;

                // Calculate the Pendulram altitude
                CurrentPendulum = RungeKutta(t, CurrentPendulum.theta, CurrentPendulum.omega);

                // Display the Pendulum
                DisplayPendulum(CurrentFulcrum.x / 10, CurrentPendulum.theta);

                return NextTimerStart;
            });
        }


        // Manually slide the Fulcrum position
        Fulcrum SlideFulcrum()
        {
            Fulcrum fulcrum = new Fulcrum();

            fulcrum.x = FulcrumSlider.Value / 10;
            fulcrum.v = (fulcrum.x - CurrentFulcrum.x) / dt;
            fulcrum.a = (fulcrum.v - CurrentFulcrum.v) / dt;

            return fulcrum;
        }


        // Reset all parameters and Initialize the Pendulum
        void ResetPendulum()
        {
            CurrentPendulum = new Pendulum();
            CurrentFulcrum = new Fulcrum();

            CurrentPendulum.theta = theta0 / 180 * Math.PI;
            FulcrumSlider.Value = 0;

            Idtheta = 0; Idx = 0;
            slowdowned = false;
            n = 1;

            DisplayPendulum(0, CurrentPendulum.theta);
        }


        // Oscillating Fulcrum for Swing up the Pendulum
        Fulcrum OscillateFulcrum(double t)
        {
            double A = 1.5;     // Amplitude
            double T = Math.Sqrt(g / CurrentPendulum.L) * 0.7;  // Period
            double omega = 2 * Math.PI / T;     // Angular frequency

            Fulcrum fulcrum = new Fulcrum()
            {
                x =  A * Math.Sin(omega * t) + X0,
                v =  A * omega * Math.Cos(omega * t),
                a = -A * Math.Pow(omega, 2) * Math.Sin(omega * t)
            };

            return fulcrum;
        }


        // Lean the Pendulum to Slow Down its horizontally move
        double SlowDownPendulum()
        {
            double theta = CurrentPendulum.theta;
            double dtheta = new double();

            // Calculate dtheta to balance the Inverted Pendulum
            if (theta >= 0) dtheta =  Math.PI - theta;
            if (theta <  0) dtheta = -Math.PI - theta;

            if (Math.Abs(CurrentFulcrum.v) < 0.1){
                slowdowned = true;
                n = 1;
            }

            if (!slowdowned){
                dtheta += 2.0 / 180.0 * Math.PI * CurrentFulcrum.v;

                // Not to largely change dtheta suddenly
                dtheta *= (double)n / 10.0; if (n < 10) n++;
            }

            return dtheta;
        }


        // Balancing the Pendulum by PID-controlling the Fulcrum position
        Fulcrum BalancePendulum(double dtheta)
        {
            Fulcrum fulcrum = new Fulcrum();

            Idtheta += (dtheta * dt);
            double v = -CurrentPendulum.omega;

            // Balancing by PID-Control
            double dx = 0.6 * Ku1 * (dtheta + Idtheta / (0.5 * Pu1) + 0.125 * Pu1 * v);
            //double dx = Kp * dtheta - Kd * CurrentPendulum.omega + Ki * Idtheta;

            // Actuate the Fulcrum position
            fulcrum.x = CurrentFulcrum.x + dx;
            fulcrum.v = (fulcrum.x - CurrentFulcrum.x) / dt;
            fulcrum.a = (fulcrum.v - CurrentFulcrum.v) / dt;

            return fulcrum;
        }


        // Lean the Pendulum to horizontally move it
        double MovePendulum(double Xd)
        {
            double theta  = CurrentPendulum.theta;
            double dtheta = new double();

            // Calculate dtheta to balance the Inverted Pendulum
            if (theta >= 0) dtheta =  Math.PI - theta;
            if (theta <  0) dtheta = -Math.PI - theta;

            double dX = CurrentFulcrum.x - Xd;
            double v  = CurrentFulcrum.v;

            Idx += (dX * dt);
            dtheta += 0.6 * Ku2 * (dX + Idx / (0.5 * Pu2) + 0.125 * Pu2 * v);

            // Not to largely change dtheta suddenly
            dtheta *= (double)n / 10.0;  if (n < 10) n++;

            return Angle(dtheta);
        }


        // When fail balancing, return the Pendulum to the center for next try
        Fulcrum ReturnPendulum()
        {
            Fulcrum fulcrum = new Fulcrum();

            if (CurrentFulcrum.x >  3.0) fulcrum.v = -10.0;
            if (CurrentFulcrum.x < -3.0) fulcrum.v =  10.0;

            fulcrum.x = CurrentFulcrum.x + fulcrum.v * 0.1;
            fulcrum.a = 0.0;

            return fulcrum;
        }


        // Calculate the Pendulum motion by 4th Order Runge-Kuta method
        Pendulum RungeKutta(double t, double theta, double omega)
        {
            Pendulum pendulum = new Pendulum();
            double k, k1, k2, k3, k4;
            double m, m1, m2, m3, m4;

            k1 = dt * F(t, theta, omega);
            m1 = dt * G(t, theta, omega);

            k2 = dt * F(t + dt / 2, theta + k1 / 2, omega + m1 / 2);
            m2 = dt * G(t + dt / 2, theta + k1 / 2, omega + m1 / 2);

            k3 = dt * F(t + dt / 2, theta + k2 / 2, omega + m2 / 2);
            m3 = dt * G(t + dt / 2, theta + k2 / 2, omega + m2 / 2);

            k4 = dt * F(t + dt, theta + k3, omega + m3);
            m4 = dt * G(t + dt, theta + k3, omega + m3);

            k = (k1 + 2 * k2 + 2 * k3 + k4) / 6;
            m = (m1 + 2 * m2 + 2 * m3 + m4) / 6;

            pendulum.theta = Angle(theta + k);
            pendulum.omega = omega + m;

            return pendulum;
        }

        // Angular Velocity Function
        double F(double t, double theta, double omega)
        {
            return omega;
        }

        // Angular Acceleration Function
        double G(double t, double theta, double omega)
        {
            double L = CurrentPendulum.L;   // The physical length of the Pendulum [m]
            double k = CurrentPendulum.k;   // Damping coefficient

            // Simple Pendulum
            //return -g / L * Math.Sin(theta);

            // Simple Pendulum with moving fulcrum
            double a = CurrentFulcrum.a;
            double v = CurrentPendulum.omega * L;   // Tangential velocity of the Pendulum

            return -g / L * Math.Sin(theta) - a / L * Math.Cos(theta) - k * v;
        }


        // Transform angle to [-PI, PI]
        double Angle(double angle)
        {
            if (angle % (2 * Math.PI) >=  Math.PI) angle -= 2 * Math.PI;
            if (angle % (2 * Math.PI) <= -Math.PI) angle += 2 * Math.PI;

            return angle;
        }


        // Start, Stop, and Reset the Pendulum Swinging
        void DetectButtonClick(object sender, EventArgs args)
        {
            Button button = (Button)sender;

            switch(button.Text)
            {
                case "Start":
                    button.Text = "Pause";
                    StopButton.Text = "Stop";
                    //control0 = control;
                    control = "SwingUp";
                    //if (control0 == "Reset") ControlPendulum();                    
                    break;
                
                case "Pause":                  
                    button.Text = "ReStart";
                    control0 = control;
                    control = "Pause";
                    break;
                
                case "ReStart":
                    button.Text = "Pause";
                    control = control0;
                    break;

                case "Stop":
                    button.Text = "Reset";
                    StartButton.Text = "Start";
                    control = "Stop";
                    break;

                case "Reset":
                    button.Text = "Stop";
                    control = "Reset";
                    break;

                case "Punch!":    // Punch the Pendulum (distarb the balancing)
                    if(button.TranslationX<150)CurrentPendulum.theta -= 0.4;
                    if(button.TranslationX>150)CurrentPendulum.theta += 0.4;
                    break;
            }
            StatusLabel.Text = "Status: " + control;
        }

    }   // End of public partial class MainPage
}       // End of namespace Pendulum

