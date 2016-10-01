using System;

namespace DynamicProgramming
{
    public static class MathHelper
    {
        public static double BilinearInterpolate(double val11, double val12, double val21, double val22,
            double x, double y)
        {
            return val11 * (1.0 - x) * (1.0 - y)
                + val21 * x * (1.0 - y)
                + val12 * (1.0 - x) * y
                + val22 * x * y;
        }

        public static double FindInterpolatedCost(double[] state, double[] inputArray, int xPoints, double xRange, int yPoints, double yRange)
        {
            // Bilinear interpolation https://en.wikipedia.org/wiki/Bilinear_interpolation

            var xIndices = FindIndicesForInterpolation(xPoints, xRange, state[0]);
            var yIndices = FindIndicesForInterpolation(yPoints, yRange, state[1]);

            var x = xIndices.Ratio;
            var y = yIndices.Ratio;

            var xLowIndex = xIndices.Lower;
            var yLowIndex = yIndices.Lower;

            var xHighIndex = xIndices.Upper;
            var yHighIndex = yIndices.Upper;

            var val11 = inputArray[(yLowIndex * xPoints) + xLowIndex];
            var val12 = inputArray[(yHighIndex * xPoints) + xLowIndex];
            var val21 = inputArray[(yLowIndex * xPoints) + xHighIndex];
            var val22 = inputArray[(yHighIndex * xPoints) + xHighIndex];

            return MathHelper.BilinearInterpolate(val11, val12, val21, val22, x, y);
        }

        public static InterpolationIndices FindIndicesForInterpolation(int numPoints, double range, double currentValue)
        {
            if (currentValue < (-range / 2.0))
            {
                return new InterpolationIndices
                {
                    Lower = 0,
                    Upper = 0,
                    Ratio = 0.0
                };
            }

            if (currentValue > (range / 2.0))
            {
                return new InterpolationIndices
                {
                    Lower = numPoints - 1,
                    Upper = numPoints - 1,
                    Ratio = 0.0
                };
            }

            var step = range / (numPoints - 1);

            var ratio = (currentValue + (range / 2.0)) / range;

            var lowerBoundPoint = (int)(ratio * (numPoints - 1));

            var ratioBetweenPoints = ((ratio * (numPoints - 1)) - lowerBoundPoint);

            // If the point falls on the upper edge, then return that
            if (lowerBoundPoint == numPoints - 1)
            {
                return new InterpolationIndices
                {
                    Lower = lowerBoundPoint,
                    Upper = lowerBoundPoint,
                    Ratio = 0.0
                };
            }

            return new InterpolationIndices
            {
                Lower = lowerBoundPoint,
                Upper = lowerBoundPoint + 1,
                Ratio = ratioBetweenPoints
            };
        }
    }

    public struct InterpolationIndices
    {
        public int Lower { get; set; }
        public int Upper { get; set; }
        public double Ratio { get; set; }
    }
}
