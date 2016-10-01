using DynamicProgramming;
using NUnit.Framework;

namespace DynamicProgrammingTest
{
    [TestFixture]
    public class InterpolatedValueTests
    {
        private double[] _cost;

        private int xPoints = 5;
        private int yPoints = 5;
        private double xRange = 10.0;
        private double yRange = 20.0;

        [SetUp]
        public void TestSetUp()
        { 
            _cost = new double[xPoints * yPoints];
            for (var x = 0; x < xPoints; ++x)
            {
                var xval = (-xRange / 2.0) + (x * (xRange / (xPoints - 1)));
                for (var y = 0; y < yPoints; ++y)
                {
                    var yval = (-yRange / 2.0) + (y * (yRange / (yPoints - 1)));

                    var cost = (xval * xval) + (yval * yval);
                    _cost[(y * xPoints) + x] = cost;
                }
            }
        }

        [Test]
        public void CentrePoint()
        {
            var interpolatedValue = MathHelper.FindInterpolatedCost(new double[] { 0.0, 0.0 },
                _cost, xPoints, xRange, yPoints, yRange);

            Assert.That(interpolatedValue, Is.EqualTo(0.0));
        }

        [Test]
        public void GridPoint()
        {
            var interpolatedValue = MathHelper.FindInterpolatedCost(new double[] { 2.5, 5.0 },
                _cost, xPoints, xRange, yPoints, yRange);

            Assert.That(interpolatedValue, Is.EqualTo((2.5 * 2.5) + (5.0 * 5.0)));
        }

        [Test]
        public void InterpolatedPoint()
        {
            var interpolatedValue = MathHelper.FindInterpolatedCost(new double[] { 1.2, 2.4 },
                _cost, xPoints, xRange, yPoints, yRange);

            Assert.That(interpolatedValue, Is.EqualTo(15.0));
        }

        [Test]
        public void InterpolatedPointXBeyondLowerBound()
        {
            var interpolatedValue = MathHelper.FindInterpolatedCost(new double[] { -10.0, 0.0 },
                _cost, xPoints, xRange, yPoints, yRange);

            Assert.That(interpolatedValue, Is.EqualTo(25.0));
        }

        [Test]
        public void InterpolatedPointXBeyondUpperBound()
        {
            var interpolatedValue = MathHelper.FindInterpolatedCost(new double[] { 10.0, 0.0 },
                _cost, xPoints, xRange, yPoints, yRange);

            Assert.That(interpolatedValue, Is.EqualTo(25.0));
        }
    }
}
