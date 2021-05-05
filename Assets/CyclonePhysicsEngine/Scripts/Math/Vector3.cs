using System;
using System.Text;

namespace Cyclone.Math
{
    public class Vector3
    {
        public double x { get; set; }

        public double y { get; set; }

        public double z { get; set; }

        public Vector3()
        {
        }

        public Vector3(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public Vector3(Vector3 vector)
        {
            x = vector.x;
            y = vector.y;
            z = vector.z;
        }

        public double this[int i]
        {
            get
            {
                if (i < 0 || i > 2)
                {
                    throw new IndexOutOfRangeException("Index " + i + " does not correspond to a Vector3 component.");
                }

                if (i == 0)
                {
                    return x;
                }

                if (i == 1)
                {
                    return y;
                }

                return z;
            }
        }

        public static Vector3 operator +(Vector3 lhs, Vector3 rhs)
        {
            return new Vector3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
        }

        public static Vector3 operator -(Vector3 lhs, Vector3 rhs)
        {
            return new Vector3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
        }

        public static Vector3 operator *(Vector3 lhs, double value)
        {
            return new Vector3(lhs.x * value, lhs.y * value, lhs.z * value);
        }

        public static bool operator ==(Vector3 lhs, Vector3 rhs)
        {
            return Core.Equals(lhs.x, rhs.x) && Core.Equals(lhs.y, rhs.y) && Core.Equals(lhs.z, rhs.z);
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

            Vector3 v = obj as Vector3;
            if ((System.Object)v == null)
            {
                return false;
            }

            return Core.Equals(x, v.x) && Core.Equals(y, v.y) && Core.Equals(z, v.z);
        }

        public bool Equals(Vector3 v)
        {
            if ((object)v == null)
            {
                return false;
            }

            return Core.Equals(x, v.x) && Core.Equals(y, v.y) && Core.Equals(z, v.z);
        }

        public static bool operator !=(Vector3 lhs, Vector3 rhs)
        {
            return !(lhs == rhs);
        }

        public static bool operator <(Vector3 lhs, Vector3 rhs)
        {
            return (lhs.x < rhs.x) && (lhs.y < rhs.y) && (lhs.z < rhs.z);
        }

        public static bool operator >(Vector3 lhs, Vector3 rhs)
        {
            return (lhs.x > rhs.x) && (lhs.y > rhs.y) && (lhs.z > rhs.z);
        }

        public Vector3 ComponentProduct(Vector3 vector)
        {
            return new Vector3(x * vector.x, y * vector.y, z * vector.z);
        }

        public void ComponentProductUpdate(Vector3 vector)
        {
            x *= vector.x;
            y *= vector.y;
            z *= vector.z;
        }

        public Vector3 VectorProduct(Vector3 vector)
        {
            return new Vector3(y * vector.z - z * vector.y,
                               z * vector.x - x * vector.z,
                               x * vector.y - y * vector.x);
        }

        public void VectorProductUpdate(Vector3 vector)
        {
            Vector3 tmp = VectorProduct(vector);
            x = tmp.x;
            y = tmp.y;
            z = tmp.z;
        }

        public double ScalarProduct(Vector3 vector)
        {
            return (x * vector.x) + (y * vector.y) + (z * vector.z);
        }

        public static double operator *(Vector3 lhs, Vector3 rhs)
        {
            return (lhs.x * rhs.x) + (lhs.y * rhs.y) + (lhs.z * rhs.z);
        }

        public void AddScaledVector(Vector3 vector, double scale)
        {
            x += vector.x * scale;
            y += vector.y * scale;
            z += vector.z * scale;
        }

        public double Magnitude
        {
            get { return System.Math.Sqrt(x * x + y * y + z * z); }
        }

        public double SquareMagnitude
        {
            get { return x * x + y * y + z * z; }
        }

        public void Trim(double size)
        {
            if (SquareMagnitude > size * size)
            {
                Normalize();
                x *= size;
                y *= size;
                z *= size;
            }
        }

        public void Normalize()
        {
            double length = Magnitude;
            if (length > 0)
            {
                x *= 1 / length;
                y *= 1 / length;
                z *= 1 / length;
            }
        }

        public Vector3 Unit()
        {
            Vector3 unit = new Vector3(this);
            unit.Normalize();
            return unit;
        }

        public void Clear()
        {
            x = y = z = 0;
        }

        public void Invert()
        {
            x = -x;
            y = -y;
            z = -z;
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            sb.AppendFormat("({0}, {1}, {2})", x, y, z);
            return sb.ToString();
        }
    }
}
