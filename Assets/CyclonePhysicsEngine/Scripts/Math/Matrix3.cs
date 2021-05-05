
namespace Cyclone.Math
{
    public class Matrix3
    {
        private const int ElementCount = 9;

        public double[] Data { get; private set; }

        public Matrix3()
        {
            Data = new double[ElementCount];
        }

        public Matrix3(Vector3 compOne, Vector3 compTwo, Vector3 compThree)
        {
            Data = new double[ElementCount];
            SetComponents(compOne, compTwo, compThree);
        }

        public Matrix3(double c0, double c1, double c2, double c3, double c4, double c5, double c6, double c7, double c8)
        {
            Data = new[] { c0, c1, c2, c3, c4, c5, c6, c7, c8 };
        }

        public Matrix3(Matrix3 other)
        {
            Data = new double[ElementCount];
            other.Data.CopyTo(Data, 0);
        }

        void SetDiagonal(double a, double b, double c)
        {
            SetInertiaTensorCoeffs(a, b, c);
        }

       public void SetInertiaTensorCoeffs(double ix, double iy, double iz, double ixy = 0, double ixz = 0, double iyz = 0)
        {
            Data[0] = ix;
            Data[1] = Data[3] = -ixy;
            Data[2] = Data[6] = -ixz;
            Data[4] = iy;
            Data[5] = Data[7] = -iyz;
            Data[8] = iz;
        }

        public void SetBlockInertiaTensor(Vector3 halfSizes, double mass)
        {
            Vector3 squares = halfSizes.ComponentProduct(halfSizes);
            SetInertiaTensorCoeffs
                (
                0.3f * mass * (squares.y + squares.z),
                0.3f * mass * (squares.x + squares.z),
                0.3f * mass * (squares.x + squares.y)
                );
        }

        public void SetSkewSymmetric(Vector3 vector)
        {
            Data[0] = Data[4] = Data[8] = 0;
            Data[1] = -vector.z;
            Data[2] = vector.y;
            Data[3] = vector.z;
            Data[5] = -vector.x;
            Data[6] = -vector.y;
            Data[7] = vector.x;
        }

        void SetComponents(Vector3 compOne, Vector3 compTwo, Vector3 compThree)
        {
            Data[0] = compOne.x;
            Data[1] = compTwo.x;
            Data[2] = compThree.x;
            Data[3] = compOne.y;
            Data[4] = compTwo.y;
            Data[5] = compThree.y;
            Data[6] = compOne.z;
            Data[7] = compTwo.z;
            Data[8] = compThree.z;
        }

        public static Vector3 operator *(Matrix3 lhs, Vector3 vector)
        {
            return new Vector3
                (
                vector.x * lhs.Data[0] + vector.y * lhs.Data[1] + vector.z * lhs.Data[2],
                vector.x * lhs.Data[3] + vector.y * lhs.Data[4] + vector.z * lhs.Data[5],
                vector.x * lhs.Data[6] + vector.y * lhs.Data[7] + vector.z * lhs.Data[8]
                );
        }

        public Vector3 Transform(Vector3 vector)
        {
            return this * vector;
        }

        public Vector3 TransformTranspose(Vector3 vector)
        {
            return new Vector3
                (
                vector.x * Data[0] + vector.y * Data[3] + vector.z * Data[6],
                vector.x * Data[1] + vector.y * Data[4] + vector.z * Data[7],
                vector.x * Data[2] + vector.y * Data[5] + vector.z * Data[8]
                );
        }

        public Vector3 GetRowVector(int i)
        {
            return new Vector3(Data[i * 3], Data[i * 3 + 1], Data[i * 3 + 2]);
        }

        public Vector3 GetAxisVector(int i)
        {
            return new Vector3(Data[i], Data[i + 3], Data[i + 6]);
        }

        public void SetInverse(Matrix3 m)
        {
            double t4 = m.Data[0] * m.Data[4];
            double t6 = m.Data[0] * m.Data[5];
            double t8 = m.Data[1] * m.Data[3];
            double t10 = m.Data[2] * m.Data[3];
            double t12 = m.Data[1] * m.Data[6];
            double t14 = m.Data[2] * m.Data[6];

            double t16 = (t4 * m.Data[8] - t6 * m.Data[7] - t8 * m.Data[8] +
                        t10 * m.Data[7] + t12 * m.Data[5] - t14 * m.Data[4]);

            if (Core.Equals(t16, 0.0))
            {
                return;
            }
            double t17 = 1 / t16;

            Data[0] = (m.Data[4] * m.Data[8] - m.Data[5] * m.Data[7]) * t17;
            Data[1] = -(m.Data[1] * m.Data[8] - m.Data[2] * m.Data[7]) * t17;
            Data[2] = (m.Data[1] * m.Data[5] - m.Data[2] * m.Data[4]) * t17;
            Data[3] = -(m.Data[3] * m.Data[8] - m.Data[5] * m.Data[6]) * t17;
            Data[4] = (m.Data[0] * m.Data[8] - t14) * t17;
            Data[5] = -(t6 - t10) * t17;
            Data[6] = (m.Data[3] * m.Data[7] - m.Data[4] * m.Data[6]) * t17;
            Data[7] = -(m.Data[0] * m.Data[7] - t12) * t17;
            Data[8] = (t4 - t8) * t17;
        }

        public Matrix3 Inverse()
        {
            Matrix3 result = new Matrix3();
            result.SetInverse(this);
            return result;
        }

        public void Invert()
        {
            Matrix3 thisMatrixCopy = new Matrix3(this);
            SetInverse(thisMatrixCopy);
        }

        void SetTranspose(Matrix3 m)
        {
            Data[0] = m.Data[0];
            Data[1] = m.Data[3];
            Data[2] = m.Data[6];
            Data[3] = m.Data[1];
            Data[4] = m.Data[4];
            Data[5] = m.Data[7];
            Data[6] = m.Data[2];
            Data[7] = m.Data[5];
            Data[8] = m.Data[8];
        }

        public Matrix3 Transpose()
        {
            Matrix3 result = new Matrix3();
            result.SetTranspose(this);
            return result;
        }

        public static Matrix3 operator *(Matrix3 lhs, Matrix3 rhs)
        {
            Matrix3 result = new Matrix3
                (
                lhs.Data[0] * rhs.Data[0] + lhs.Data[1] * rhs.Data[3] + lhs.Data[2] * rhs.Data[6],
                lhs.Data[0] * rhs.Data[1] + lhs.Data[1] * rhs.Data[4] + lhs.Data[2] * rhs.Data[7],
                lhs.Data[0] * rhs.Data[2] + lhs.Data[1] * rhs.Data[5] + lhs.Data[2] * rhs.Data[8],

                lhs.Data[3] * rhs.Data[0] + lhs.Data[4] * rhs.Data[3] + lhs.Data[5] * rhs.Data[6],
                lhs.Data[3] * rhs.Data[1] + lhs.Data[4] * rhs.Data[4] + lhs.Data[5] * rhs.Data[7],
                lhs.Data[3] * rhs.Data[2] + lhs.Data[4] * rhs.Data[5] + lhs.Data[5] * rhs.Data[8],

                lhs.Data[6] * rhs.Data[0] + lhs.Data[7] * rhs.Data[3] + lhs.Data[8] * rhs.Data[6],
                lhs.Data[6] * rhs.Data[1] + lhs.Data[7] * rhs.Data[4] + lhs.Data[8] * rhs.Data[7],
                lhs.Data[6] * rhs.Data[2] + lhs.Data[7] * rhs.Data[5] + lhs.Data[8] * rhs.Data[8]
                );

            return result;
        }

        public static Matrix3 operator *(Matrix3 lhs, double scalar)
        {
            Matrix3 result = new Matrix3(lhs);

            result.Data[0] *= scalar; result.Data[1] *= scalar; result.Data[2] *= scalar;
            result.Data[3] *= scalar; result.Data[4] *= scalar; result.Data[5] *= scalar;
            result.Data[6] *= scalar; result.Data[7] *= scalar; result.Data[8] *= scalar;

            return result;
        }

        public static Matrix3 operator +(Matrix3 lhs, Matrix3 rhs)
        {
            Matrix3 result = new Matrix3(lhs);

            result.Data[0] += rhs.Data[0]; result.Data[1] += rhs.Data[1]; result.Data[2] += rhs.Data[2];
            result.Data[3] += rhs.Data[3]; result.Data[4] += rhs.Data[4]; result.Data[5] += rhs.Data[5];
            result.Data[6] += rhs.Data[6]; result.Data[7] += rhs.Data[7]; result.Data[8] += rhs.Data[8];

            return result;
        }

        public void SetOrientation(Quaternion q)
        {
            Data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
            Data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
            Data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
            Data[3] = 2 * q.i * q.j - 2 * q.k * q.r;
            Data[4] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
            Data[5] = 2 * q.j * q.k + 2 * q.i * q.r;
            Data[6] = 2 * q.i * q.k + 2 * q.j * q.r;
            Data[7] = 2 * q.j * q.k - 2 * q.i * q.r;
            Data[8] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
        }

        public static Matrix3 LinearInterpolate(Matrix3 a, Matrix3 b, double prop)
        {
            Matrix3 result = new Matrix3();
            for (int i = 0; i < ElementCount; i++)
            {
                result.Data[i] = a.Data[i] * (1 - prop) + b.Data[i] * prop;
            }
            return result;
        }

        public static bool operator ==(Matrix3 lhs, Matrix3 rhs)
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

            Matrix3 m = obj as Matrix3;
            if ((System.Object)m == null)
            {
                return false;
            }

            return (this == m);
        }

        public bool Equals(Matrix3 m)
        {
            if ((object)m == null)
            {
                return false;
            }

            return (this == m);
        }

        public static bool operator !=(Matrix3 lhs, Matrix3 rhs)
        {
            return !(lhs == rhs);
        }
    }
}
