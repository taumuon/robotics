using System;
using System.Linq;

namespace DynamicProgramming
{
    public class ValueIteration
    {
        public static void ValueIterationStep(
            double [] finalCosts,
            double [] currentCosts,
            double [] newCosts,
            double [] newControl,
            double [][][] updatedStatesPerControl,
            double [] discretisedControl,
            double gamma,
            Func<double[], double, double> costPerStepFunc,
            int xPoints,
            int yPoints,
            double xRange,
            double yRange)
        {
            var dataLength = xPoints * yPoints;

            for (var itemIndex = 0; itemIndex < dataLength; ++itemIndex)
            {
                var cost = currentCosts[itemIndex];

                var lowestCost = double.MaxValue;
                var lowestControl = double.MaxValue;

                // Arg-min over the control
                for (var controlIndex = 0; controlIndex < discretisedControl.Length; ++controlIndex)
                {
                    var u = discretisedControl[controlIndex];

                    var newState = updatedStatesPerControl[controlIndex][itemIndex];

                    var costPerStep = costPerStepFunc(newState, u);

                    // var interpolatedCost = MathHelper.FindInterpolatedCost(newState, currentCosts, xPoints, xRange, yPoints, yRange);
                    // var newStepCost = 0.95 * interpolatedCost;

                    var interpolatedCost = MathHelper.FindInterpolatedCost(newState, currentCosts, xPoints, xRange, yPoints, yRange);
                    var newStepCost = costPerStep + (gamma * interpolatedCost);

                    if (newStepCost < lowestCost)
                    {
                        lowestControl = u;
                        lowestCost = newStepCost;
                    }
                }

                // newCosts[itemIndex] = (gamma * cost) + (0.1 * lowestCost);
                newCosts[itemIndex] = lowestCost;
                newControl[itemIndex] = lowestControl;
            }
        }

        public static Tuple<double[], double[], int> ValueIterate(
            double[] finalCost,
            double[] discretisedControl,
            Func<double[], double, double, double[]> stateEquation,
            int xPoints,
            int yPoints,
            double xRange,
            double yRange,
            int maxIterations = 5000)
        {
            var dataLength = xPoints * yPoints;

            var currentCosts = finalCost.ToArray();
            var currentControls = new double[dataLength];

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

            var newCosts = new double[dataLength];
            var newControl = new double[dataLength];

            // https://courses.cs.washington.edu/courses/cse473/12sp/slides/16-mdp.pdf
            // slide 6 convergence
            // max norm ||U|| = max over s |U(s)|
            //  then ||U_t+1 - U_t|| << epsilon
            //  => ||U_t+1-U|| < 2 * epsilon * gamma / (1 - gamma)

            var gamma = 0.999;

            var norm = double.MaxValue;
            var tolerance = 0.1;
            int count = 0;

            var xTolerance = xRange / xPoints;
            var yTolerance = yRange / yPoints;

            // Minimum time control. U is not used (would be for LQR step)
            Func<double[], double, double> costPerStepFunc =
                (s, u) => Math.Abs(s[0]) < xTolerance && Math.Abs(s[1]) < yTolerance
                ? 0.0 : 1.0;

            while (norm > tolerance && count < maxIterations)
            {
                count++;

                ValueIterationStep(finalCost, currentCosts, newCosts, newControl,
                    updatedStatesPerControl, discretisedControl, gamma,
                    costPerStepFunc,
                    xPoints, yPoints, xRange, yRange);

                norm = MaxNorm(currentCosts, newCosts);

                Array.Copy(newCosts, currentCosts, newCosts.Length);
                Array.Copy(newControl, currentControls, newControl.Length);
            }

            return Tuple.Create(currentCosts, currentControls, count);
        }

        private static double MaxNorm(double[] arr1, double[] arr2)
        {
            // TODO: use Numerics.Vector/SIMD
            return arr1.Zip(arr2, (x, y) => Math.Abs(x - y)).Max();
        }

        private static double StateValueForIndex(int numPoints, int currentPoint, double range)
        {
            var step = range / (numPoints - 1);
            return (-range / 2.0) + (step * currentPoint);
        }
    }
}
