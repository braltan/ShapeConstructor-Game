using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Cyclone.Math
{
    public class Quaternion
    {
        public double r { get; set; }

        public double i { get; set; }

        public double j { get; set; }

        public double k { get; set; }

        public Quaternion()
        {
            r = 0;
            i = 0;
            j = 0;
            k = 0;
        }

        public Quaternion(double r, double i, double j, double k)
        {
            this.r = r;
            this.i = i;
            this.j = j;
            this.k = k;
        }

        public Quaternion(Quaternion other)
        {
            r = other.r;
            i = other.i;
            j = other.j;
            k = other.k;
        }

        public void Normalize()
        {
            double d = r * r + i * i + j * j + k * k;

            if (Core.Equals(d, 0.0))
            {
                r = 1;
                return;
            }

            d = (1.0) / System.Math.Sqrt(d);
            r *= d;
            i *= d;
            j *= d;
            k *= d;
        }

        public static Quaternion operator *(Quaternion lhs, Quaternion rhs)
        {
            Quaternion q = new Quaternion();
            q.r = lhs.r * rhs.r - lhs.i * rhs.i -
                  lhs.j * rhs.j - lhs.k * rhs.k;
            q.i = lhs.r * rhs.i + lhs.i * rhs.r +
                  lhs.j * rhs.k - lhs.k * rhs.j;
            q.j = lhs.r * rhs.j + lhs.j * rhs.r +
                  lhs.k * rhs.i - lhs.i * rhs.k;
            q.k = lhs.r * rhs.k + lhs.k * rhs.r +
                  lhs.i * rhs.j - lhs.j * rhs.i;

            return q;
        }

        public void AddScaledVector(Vector3 vector, double scale)
        {
            Quaternion q = new Quaternion
                (
                0,
                vector.x * scale,
                vector.y * scale,
                vector.z * scale
                );

            q *= this;
            r += q.r * 0.5;
            i += q.i * 0.5;
            j += q.j * 0.5;
            k += q.k * 0.5;
        }

        public void RotateByVector(Vector3 vector)
        {
            Quaternion q = new Quaternion(0, vector.x, vector.y, vector.z);
            Quaternion thisQuaternion = this;
            thisQuaternion *= q;
            r = thisQuaternion.r;
            i = thisQuaternion.i;
            j = thisQuaternion.j;
            k = thisQuaternion.k;
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            sb.AppendFormat("({0}, {1}, {2}, {3})", i, j, k, r);
            return sb.ToString();
        }
    }
}
