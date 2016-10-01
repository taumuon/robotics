using DynamicProgramming;
using NUnit.Framework;

namespace DynamicProgrammingTest
{
    [TestFixture]
    public class FindInterpolationIndicesTests
    {
        [Test]
        public void CentrePointIndex()
        {
            var result = MathHelper.FindIndicesForInterpolation(5, 10.0, 0.0);
            Assert.That(result.Lower, Is.EqualTo(2));
            Assert.That(result.Upper, Is.EqualTo(3));
            Assert.That(result.Ratio, Is.EqualTo(0.0));
        }

        [Test]
        public void FirstPointIndex()
        {
            var result = MathHelper.FindIndicesForInterpolation(5, 10.0, -2.5);
            Assert.That(result.Lower, Is.EqualTo(1));
            Assert.That(result.Upper, Is.EqualTo(2));
            Assert.That(result.Ratio, Is.EqualTo(0.0));
        }

        [Test]
        public void InterpolatedPointIndex()
        {
            var result = MathHelper.FindIndicesForInterpolation(5, 10.0, -4.0);
            Assert.That(result.Lower, Is.EqualTo(0));
            Assert.That(result.Upper, Is.EqualTo(1));
            Assert.That(result.Ratio, Is.EqualTo(0.4));
        }

        [Test]
        public void InterpolatedPointIndex2()
        {
            var result = MathHelper.FindIndicesForInterpolation(5, 10.0, 3.0);
            Assert.That(result.Lower, Is.EqualTo(3));
            Assert.That(result.Upper, Is.EqualTo(4));
            Assert.That(EqualWithinTolerance(result.Ratio, 0.2), Is.True);
        }

        [Test]
        public void TestPointThatIsWithinNumericalTolerance()
        {
            const int xPoints = 49;
            const double xRange = 20.0;

            var xStateValue = -10.416666666;

            var result = MathHelper.FindIndicesForInterpolation(xPoints, xRange, xStateValue);
            Assert.That(result.Lower, Is.EqualTo(0));
            Assert.That(result.Upper, Is.EqualTo(0));
            Assert.That(EqualWithinTolerance(result.Ratio, 0.0), Is.True);
        }

        private static double StateValueForIndex(int numPoints, int currentPoint, double range)
        {
            var step = range / (numPoints - 1);
            return (-range / 2.0) + (step * currentPoint);
        }

        private bool EqualWithinTolerance(double val1, double val2)
        {
            return System.Math.Abs(val1 - val2) < 1e-6;
        }
    }
}
