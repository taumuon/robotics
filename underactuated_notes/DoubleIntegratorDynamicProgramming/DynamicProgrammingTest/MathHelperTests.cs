using DynamicProgramming;
using NUnit.Framework;

namespace DynamicProgrammingTest
{
    [TestFixture]
    public class MathHelperTests
    {
        [Test]
        public void TestBilinearInterpolate()
        {
            var val11 = 91.0;
            var val12 = 162.0;
            var val21 = 210.0;
            var val22 = 95.0;

            var interpolated = MathHelper.BilinearInterpolate(val11, val12, val21, val22, 0.5, 0.2);

            Assert.That(interpolated, Is.EqualTo(146.1));
        }
    }
}
