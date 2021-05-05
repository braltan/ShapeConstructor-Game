using System;
using System.Collections.Generic;
using System.Linq;

namespace Cyclone.Math
{
    public class Matrix4
    {
        private const int ElementCount = 12;

        public double[] Data { get; private set; }

        public Matrix4()
        {
            Data = new double[ElementCount];
            Data[0] = Data[5] = Data[10] = 1;
        }

        public Matrix4
            (
            double c0, double c1, double c2, double c3,
            double c4, double c5, double c6, double c7,
            double c8, double c9, double c10, double c11
            )
        {
            Data = new[] { c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11 };
        }

        public Matrix4(List<double> values)
        {
            if (values.Count != ElementCount)
            {
                throw new Exception("not enough values to initialize this matrix");
            }
            Data = values.ToArray();
        }

        public Matrix4(double[] values)
        {
            if (values.Length != ElementCount)
            {
                throw new Exception("not enough values to initialize this matrix");
            }
            Data = new double[ElementCount];
            values.CopyTo(Data, 0);
        }

        public Matrix4(Matrix4 other)
        {
            Data = new double[ElementCount];
            other.Data.CopyTo(Data, 0);
        }

        public void SetDiagonal(double a, double b, double c)
        {
            Data[0] = a;
            Data[5] = b;
            Data[10] = c;
        }

        public static Matrix4 operator *(Matrix4 lhs, Matrix4 rhs)
        {
            Matrix4 result = new Matrix4();
            result.Data[0] = (rhs.Data[0] * lhs.Data[0]) + (rhs.Data[4] * lhs.Data[1]) + (rhs.Data[8] * lhs.Data[2]);
            result.Data[4] = (rhs.Data[0] * lhs.Data[4]) + (rhs.Data[4] * lhs.Data[5]) + (rhs.Data[8] * lhs.Data[6]);
            result.Data[8] = (rhs.Data[0] * lhs.Data[8]) + (rhs.Data[4] * lhs.Data[9]) + (rhs.Data[8] * lhs.Data[10]);

            result.Data[1] = (rhs.Data[1] * lhs.Data[0]) + (rhs.Data[5] * lhs.Data[1]) + (rhs.Data[9] * lhs.Data[2]);
            result.Data[5] = (rhs.Data[1] * lhs.Data[4]) + (rhs.Data[5] * lhs.Data[5]) + (rhs.Data[9] * lhs.Data[6]);
            result.Data[9] = (rhs.Data[1] * lhs.Data[8]) + (rhs.Data[5] * lhs.Data[9]) + (rhs.Data[9] * lhs.Data[10]);

            result.Data[2] = (rhs.Data[2] * lhs.Data[0]) + (rhs.Data[6] * lhs.Data[1]) + (rhs.Data[10] * lhs.Data[2]);
            result.Data[6] = (rhs.Data[2] * lhs.Data[4]) + (rhs.Data[6] * lhs.Data[5]) + (rhs.Data[10] * lhs.Data[6]);
            result.Data[10] = (rhs.Data[2] * lhs.Data[8]) + (rhs.Data[6] * lhs.Data[9]) + (rhs.Data[10] * lhs.Data[10]);

            result.Data[3] = (rhs.Data[3] * lhs.Data[0]) + (rhs.Data[7] * lhs.Data[1]) + (rhs.Data[11] * lhs.Data[2]) + lhs.Data[3];
            result.Data[7] = (rhs.Data[3] * lhs.Data[4]) + (rhs.Data[7] * lhs.Data[5]) + (rhs.Data[11] * lhs.Data[6]) + lhs.Data[7];
            result.Data[11] = (rhs.Data[3] * lhs.Data[8]) + (rhs.Data[7] * lhs.Data[9]) + (rhs.Data[11] * lhs.Data[10]) + lhs.Data[11];

            return result;
        }

        public static Vector3 operator *(Matrix4 lhs, Vector3 vector)
        {
            return new Vector3
                (
                vector.x * lhs.Data[0] +
                vector.y * lhs.Data[1] +
                vector.z * lhs.Data[2] + lhs.Data[3],

                vector.x * lhs.Data[4] +
                vector.y * lhs.Data[5] +
                vector.z * lhs.Data[6] + lhs.Data[7],

                vector.x * lhs.Data[8] +
                vector.y * lhs.Data[9] +
                vector.z * lhs.Data[10] + lhs.Data[11]
                );
        }

        public Vector3 Transform(Vector3 vector)
        {
            return this * vector;
        }

        public double GetDeterminant()
        {
            return -Data[2] * Data[5] * Data[8] +
                    Data[1] * Data[6] * Data[8] +
                    Data[2] * Data[4] * Data[9] -
                    Data[0] * Data[6] * Data[9] -
                    Data[1] * Data[4] * Data[10] +
                    Data[0] * Data[5] * Data[10];
        }

        public void SetInverse(Matrix4 m)
        {
            double det = m.GetDeterminant();
            if (Core.Equals(det, 0.0))
            {
                return;
            }
            det = (1.0) / det;

            Data[0] = (-m.Data[9] * m.Data[6] + m.Data[5] * m.Data[10]) * det;
            Data[4] = (m.Data[8] * m.Data[6] - m.Data[4] * m.Data[10]) * det;
            Data[8] = (-m.Data[8] * m.Data[5] + m.Data[4] * m.Data[9]) * det;

            Data[1] = (m.Data[9] * m.Data[2] - m.Data[1] * m.Data[10]) * det;
            Data[5] = (-m.Data[8] * m.Data[2] + m.Data[0] * m.Data[10]) * det;
            Data[9] = (m.Data[8] * m.Data[1] - m.Data[0] * m.Data[9]) * det;

            Data[2] = (-m.Data[5] * m.Data[2] + m.Data[1] * m.Data[6]) * det;
            Data[6] = (+m.Data[4] * m.Data[2] - m.Data[0] * m.Data[6]) * det;
            Data[10] = (-m.Data[4] * m.Data[1] + m.Data[0] * m.Data[5]) * det;

            Data[3] = (m.Data[9] * m.Data[6] * m.Data[3]
                       - m.Data[5] * m.Data[10] * m.Data[3]
                       - m.Data[9] * m.Data[2] * m.Data[7]
                       + m.Data[1] * m.Data[10] * m.Data[7]
                       + m.Data[5] * m.Data[2] * m.Data[11]
                       - m.Data[1] * m.Data[6] * m.Data[11]) * det;
            Data[7] = (-m.Data[8] * m.Data[6] * m.Data[3]
                       + m.Data[4] * m.Data[10] * m.Data[3]
                       + m.Data[8] * m.Data[2] * m.Data[7]
                       - m.Data[0] * m.Data[10] * m.Data[7]
                       - m.Data[4] * m.Data[2] * m.Data[11]
                       + m.Data[0] * m.Data[6] * m.Data[11]) * det;
            Data[11] = (m.Data[8] * m.Data[5] * m.Data[3]
                       - m.Data[4] * m.Data[9] * m.Data[3]
                       - m.Data[8] * m.Data[1] * m.Data[7]
                       + m.Data[0] * m.Data[9] * m.Data[7]
                       + m.Data[4] * m.Data[1] * m.Data[11]
                       - m.Data[0] * m.Data[5] * m.Data[11]) * det;
        }

        public Matrix4 Inverse()
        {
            Matrix4 result = new Matrix4();
            result.SetInverse(this);
            return result;
        }

        public void Invert()
        {
            Matrix4 thisMatrixCopy = new Matrix4(this);
            SetInverse(thisMatrixCopy);
        }

        public Vector3 TransformDirection(Vector3 vector)
        {
            return new Vector3
               (
                vector.x * Data[0] +
                vector.y * Data[1] +
                vector.z * Data[2],

                vector.x * Data[4] +
                vector.y * Data[5] +
                vector.z * Data[6],

                vector.x * Data[8] +
                vector.y * Data[9] +
                vector.z * Data[10]
                );
        }

        public Vector3 TransformInverseDirection(Vector3 vector)
        {
            return new Vector3
                (
                vector.x * Data[0] +
                vector.y * Data[4] +
                vector.z * Data[8],

                vector.x * Data[1] +
                vector.y * Data[5] +
                vector.z * Data[9],

                vector.x * Data[2] +
                vector.y * Data[6] +
                vector.z * Data[10]
                );
        }

        public Vector3 TransformInverse(Vector3 vector)
        {
            Vector3 tmp = vector;
            tmp.x -= Data[3];
            tmp.y -= Data[7];
            tmp.z -= Data[11];
            return new Vector3
                (
                tmp.x * Data[0] +
                tmp.y * Data[4] +
                tmp.z * Data[8],

                tmp.x * Data[1] +
                tmp.y * Data[5] +
                tmp.z * Data[9],

                tmp.x * Data[2] +
                tmp.y * Data[6] +
                tmp.z * Data[10]
                );
        }

        public Vector3 GetAxisVector(int i)
        {
            return new Vector3(Data[i], Data[i + 4], Data[i + 8]);
        }

        public void SetOrientationAndPos(Quaternion q, Vector3 pos)
        {
            Data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
            Data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
            Data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
            Data[3] = pos.x;

            Data[4] = 2 * q.i * q.j - 2 * q.k * q.r;
            Data[5] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
            Data[6] = 2 * q.j * q.k + 2 * q.i * q.r;
            Data[7] = pos.y;

            Data[8] = 2 * q.i * q.k + 2 * q.j * q.r;
            Data[9] = 2 * q.j * q.k - 2 * q.i * q.r;
            Data[10] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
            Data[11] = pos.z;
        }

        public static bool operator ==(Matrix4 lhs, Matrix4 rhs)
        {
            for (int i = 0; i < lhs.Data.Length; i++)
            {
                if (!Core.Equals(lhs.Data[i], rhs.Data[i]))
                {
                    return false;
                }
            }

            return true;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        public override bool Equals(System.Object obj)
        {
            if (obj == null)
            {
                return false;
            }

            Matrix4 m = obj as Matrix4;
            if ((System.Object)m == null)
            {
                return false;
            }

            return (this == m);
        }

        public bool Equals(Matrix4 m)
        {
            if ((object)m == null)
            {
                return false;
            }

            return (this == m);
        }

        public static bool operator !=(Matrix4 lhs, Matrix4 rhs)
        {
            return !(lhs == rhs);
        }
    }
}
