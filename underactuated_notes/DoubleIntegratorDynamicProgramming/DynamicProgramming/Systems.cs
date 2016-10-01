using System;

namespace DynamicProgramming
{
    public class Systems
    {
        public static Func<double[], double, double, double[]> DoubleIntegrator = (state, deltaTime, u) =>
        {
            var x = state[0];
            var x_dot = state[1];

            return new double[] { x + (x_dot * deltaTime), x_dot + (u * deltaTime) };
        };
    }
}
