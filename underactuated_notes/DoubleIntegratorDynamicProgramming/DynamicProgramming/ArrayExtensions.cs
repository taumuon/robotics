using System;

namespace DynamicProgramming
{
    public static class ArrayEx2
    {
        // TODO: could take an expression? then could even auto-SIMD or OpenCL ?
        // TODO: is there a form of iter that doesn't actually iterate over the array, but
        //  just generates the required indices?
        public static U[] Mapi<T, U>(this T[] input, int xPoints, int yPoints, Func<int, int, T, U> iter)
        {
            var len = xPoints * yPoints;

            var output = new U[len];

            for (var y = 0; y < yPoints; y++)
            {
                for (var x = 0; x < xPoints; x++)
                {
                    var index = (y * xPoints) + x;
                    output[index] = iter(x, y, input[index]);
                }
            }

            return output;
        }

        /// <summary>
        /// Iterates over a flat array and populates as if a multi-dimensional array
        /// </summary>
        public static T[] Init<T>(int xPoints, int yPoints, Func<int, int, T> iter)
        {
            var len = xPoints * yPoints;

            var output = new T[len];

            for (var y = 0; y < yPoints; y++)
            {
                for (var x = 0; x < xPoints; x++)
                {
                    output[(y * xPoints) + x] = iter(x, y);
                }
            }

            return output;
        }
    }
}
