using DynamicProgramming;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;

namespace DIDP
{
    // TODO: 4-state, cart pole, double pendulum, acrobot
    // 6-state. triple pendulum, double pendulum on cart
    //   can investigate control discretised by more than one dimension (e.g. motor on two of three joints)

    class Program
    {
        private static double StateValueForIndex(int numPoints, int currentPoint, double range)
        {
            var step = range / (numPoints - 1);
            return (-range / 2.0) + (step * currentPoint);
        }

        private static double[] GetFinalCost(int xPoints, int yPoints, double xRange, double yRange)
        {
            double[] finalCost = new double[xPoints * yPoints];
            for(var i = 0; i < finalCost.Length; ++i) { finalCost[i] = 1.0; }

            var centreIndexX = xPoints / 2;
            var centreIndexY = yPoints / 2;

            finalCost[(xPoints * centreIndexY) + centreIndexX] = 0.0;
            if (xPoints % 2 != 0)
            {
                finalCost[(xPoints * centreIndexY) + (centreIndexX + 1)] = 0.0;
                if (yPoints % 2 != 0)
                {
                    finalCost[(xPoints * (centreIndexY + 1)) + (centreIndexX + 1)] = 0.0;
                }
            }
            else if (yPoints % 2 != 0)
            {
                finalCost[(xPoints * (centreIndexY + 1)) + centreIndexX] = 0.0;
            }

            return finalCost;
        }

        const int xPoints = 199;
        const int yPoints = 199;
        const double xRange = 10.0;
        const double yRange = 5.0;
        //const int xPoints = 49;
        //const int yPoints = 49;
        //const double xRange = 20.0;
        //const double yRange = 10.0;

        static void Main(string[] args)
        {
            AnimateValueIteration();
            PlotTrajectory();
        }

        private static void AnimateValueIteration()
        {
            double[] finalCost = GetFinalCost(xPoints, yPoints, xRange, yRange);

            var discretisedControl = new double[] { -1.0, 0.0, 1.0 };

            Func<double[], double, double, double[]> stateEquation = (state, u, t) => Systems.DoubleIntegrator(state, u, t);

            // run the ValueIterate loop manually, for plotting

            var stateValues = ArrayEx2.Init(xPoints, yPoints,
                                                (x, y) =>
                                                    new double[]
                                                    {
                                                                    StateValueForIndex(xPoints, x, xRange),
                                                                    StateValueForIndex(yPoints, y, yRange)
                                                    }
                                            );

            var updatedStatesPerControl = discretisedControl
                .Select(u => stateValues.Select(state => stateEquation(state, 0.01, u)).ToArray())
                .ToArray();

            var size = xPoints * yPoints;
            var currentCost = finalCost.ToArray();
            var newCosts = new double[size];
            var newControl = new double[size];
            var gamma = 0.999;
            var plotEveryN = 10;

            var xTolerance = xRange / xPoints;
            var yTolerance = yRange / yPoints;

            // Minimum time control. U is not used (would be for LQR step)
            Func<double[], double, double> costPerStepFunc =
                (s, u) => Math.Abs(s[0]) < xTolerance && Math.Abs(s[1]) < yTolerance
                ? 0.0 : 1.0;

            for (var count = 0; count < 1000; ++count)
            {
                if (count % plotEveryN == 0)
                {
                    var filenameNumberString = (count / plotEveryN).ToString("D3");

                    var controlPath = string.Format("E:/temp/plot/control/{0}.png", filenameNumberString);
                    var costPath = string.Format("E:/temp/plot/cost/{0}.png", filenameNumberString);
                    Write2DPlot(controlPath, "control", newControl, xPoints, yPoints, xRange, yRange);
                    Write2DPlot(costPath, "cost", currentCost, xPoints, yPoints, xRange, yRange);
                }

                ValueIteration.ValueIterationStep(finalCost, currentCost, newCosts, newControl,
                    updatedStatesPerControl, discretisedControl, gamma,
                    costPerStepFunc,
                    xPoints, yPoints, xRange, yRange);

                if (count % plotEveryN == 0)
                {
                    var maxDiff = newCosts.Zip(currentCost, (x, y) => Math.Abs(x - y)).Max();
                    Console.WriteLine(count.ToString("D3") + "  " + maxDiff.ToString());
                }

                Array.Copy(newCosts, currentCost, newCosts.Length);
            }
        }

        private static double AnalyticalDoubleIntegratorBangBangControl(double[] state)
        {
            var x = state[0];
            var v = state[1];

            var switchingSurface = -Math.Sign(x) * Math.Sqrt(2 * Math.Sign(x) * x);
            double control = 0.0;
            if (v < (switchingSurface - 1e-4))
            {
                control = 1;
            }
            else if (v > (switchingSurface + 1e-4))
            {
                control = -1;
            }
            return control;
        }

        private static void PlotTrajectory()
        {
            double[] finalCost = GetFinalCost(xPoints, yPoints, xRange, yRange);

            var discretisedControl = new double[] { -1.0, 0.0, 1.0 };

            Func<double[], double, double, double[]> stateEquation = (s, u, t) => Systems.DoubleIntegrator(s, u, t);

            var result = ValueIteration.ValueIterate(finalCost,
                discretisedControl,
                stateEquation,
                xPoints,
                yPoints,
                xRange,
                yRange,
                10000);

            var currentCost = result.Item1;
            var controls = result.Item2;
            var count = result.Item3;
            Console.WriteLine("Converged in {0} iterations", count);

            // initial state at x = -2.0, heading in -ve x direction with v = -1.5 units/sec
            var initialState = new double[] { -2.0, -1.5 };

            var state = initialState.ToArray();
            const int numIterations = 2000;
            List<double[]> statesAndControls = new List<double[]>();
            for (var iteration = 0; iteration < numIterations; iteration++)
            {
                var control = MathHelper.FindInterpolatedCost(state, controls, xPoints, xRange, yPoints, yRange);
                // var control = AnalyticalDoubleIntegratorBangBangControl(state);

                state = Systems.DoubleIntegrator(state, 0.01, control);
                if (iteration % 10 == 0)
                {
                    statesAndControls.Add(new[] { state[0], state[1], control });
                }
            }

            // liuxingguang.blogspot.co.uk/how-to-call-gnuplot-from-c-wpf.html
            // msdn.microsoft.com/en-us/library/hh297126(v=vs.100).aspx
            using (var process = new Process())
            {
                process.StartInfo.FileName = _pathToGnuPlot;
                process.StartInfo.CreateNoWindow = true;
                process.StartInfo.UseShellExecute = false;
                process.StartInfo.RedirectStandardInput = true;
                process.Start();
                using (StreamWriter sw = process.StandardInput)
                {
                    sw.WriteLine("set view map");
                    // sw.WriteLine("set dgrid3d");

                    sw.WriteLine("set terminal png size 640,480");
                    var outputFileName = "E:/temp/plot/trajectory.png";
                    var title = "trajectory";
                    var outputCommand = "set output " + '"' + outputFileName + '"';
                    sw.WriteLine(outputCommand);

                    sw.WriteLine("set title \"" + title + "\"");
                    sw.WriteLine("set xlabel \"x\"");
                    sw.WriteLine("set ylabel \"v\"");

                    sw.WriteLine("set style data lines");

                    // with lines lc rgb 'blue'
                    // var inputText = "splot '-' using 1:2:3 with lines ls 1" + "\n";
                    var inputText = "splot '-' using 1:2:3 with lines palette" + "\n";

                    sw.WriteLine(inputText);

                    foreach(var stateAndControl in statesAndControls)
                    {
                        var xStateValue = stateAndControl[0];
                        var vStateValue = stateAndControl[1];
                        var control = stateAndControl[2];

                        sw.WriteLine(string.Format("{0} {1} {2}", xStateValue, vStateValue, control));
                    }

                    sw.WriteLine(" e "); // terminate data
                    sw.Flush();
                }
            }
        }

        private const string _pathToGnuPlot = @"C:\Program Files (x86)\gnuplot\bin\gnuplot.exe";

        private static void Write2DPlot(string outputFileName, string title, double[] array, int xPoints, int yPoints, double xRange, double yRange)
        {
            // liuxingguang.blogspot.co.uk/how-to-call-gnuplot-from-c-wpf.html
            // msdn.microsoft.com/en-us/library/hh297126(v=vs.100).aspx
            using (var process = new Process())
            {
                process.StartInfo.FileName = _pathToGnuPlot;
                process.StartInfo.CreateNoWindow = true;
                process.StartInfo.UseShellExecute = false;
                process.StartInfo.RedirectStandardInput = true;
                process.Start();
                using (StreamWriter sw = process.StandardInput)
                {
                    sw.WriteLine("set view map");
                    //sw.WriteLine("set dgrid3d");
                    
                    sw.WriteLine("set terminal png size 640,480");
                    var outputCommand = "set output " + '"' + outputFileName + '"';
                    sw.WriteLine(outputCommand);

                    sw.WriteLine("set title \"" + title + "\"");
                    sw.WriteLine("set xlabel \"x\"");
                    sw.WriteLine("set ylabel \"v\"");

                    var inputText = "plot '-' using 1:2:3 with image pixels" + "\n";

                    //sw.WriteLine("set pm3d");
                    // sw.WriteLine("set pm3d interpolate 0,0");
                    //sw.WriteLine("set palette maxcolors 3");
                    //sw.WriteLine("set palette defined 0 '#000fff', 1 '#90ff70', 2 '#ee0000'");
                    //var inputText = "splot '-' using 1:2:3 with pm3d" + "\n";

                    sw.WriteLine(inputText);

                    var length = xPoints * yPoints;
                    var stateValues = ArrayEx2.Init(xPoints, yPoints,
                        (x, y) =>
                            new double[]
                            {
                                StateValueForIndex(xPoints, x, xRange),
                                StateValueForIndex(yPoints, y, yRange)
                            }
                    );

                    for (int index = 0; index < length; index++)
                    {
                        var stateValue = stateValues[index];
                        var xStateValue = stateValue[0];
                        var yStateValue = stateValue[1];

                        var plotValue = array[index];

                        sw.WriteLine(string.Format("{0} {1} {2}", xStateValue, yStateValue, plotValue));
                    }

                    sw.WriteLine(" e "); // terminate data
                    sw.Flush();
                }
            }
        }
    }
}
