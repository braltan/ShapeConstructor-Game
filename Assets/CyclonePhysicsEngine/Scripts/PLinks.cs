using System;
using Cyclone.Math;

namespace Cyclone
{
    public abstract class ParticleContactGenerator
    {
        public abstract int AddContact(ParticleContact contact, int limit);
    }

    public class ParticleCable : ParticleContactGenerator
    {
        public Particle[] particle;

        public double MaxLength { get; set; }

        public double Restitution { get; set; }

        protected double CurrentLength()
        {
            Vector3 relativePos = particle[0].GetPosition() - particle[1].GetPosition();
            return relativePos.Magnitude;
        }

        public override int AddContact(ParticleContact contact, int limit)
        {
            double length = CurrentLength();

            if(length < MaxLength)
            {
                return 0;
            }

            contact.particle[0] = particle[0];
            contact.particle[1] = particle[1];

            Vector3 normal = particle[1].GetPosition() - particle[0].GetPosition();
            normal.Normalize();
            contact.ContactNormal = normal;

            contact.Penetration = length - MaxLength;
            contact.Restitution = Restitution;

            return 1;
        }
    }

    public class ParticleRod : ParticleContactGenerator
    {
        public Particle[] particle;

        public double Length { get; set; }

        protected double CurrentLength()
        {
            Vector3 relativePos = particle[0].GetPosition() - particle[1].GetPosition();
            return relativePos.Magnitude;
        }

        public override int AddContact(ParticleContact contact, int limit)
        {
            double length = CurrentLength();

            if (length == Length)
            {
                return 0;
            }

            contact.particle[0] = particle[0];
            contact.particle[1] = particle[1];

            Vector3 normal = particle[1].GetPosition() - particle[0].GetPosition();
            normal.Normalize();

            if(length > Length)
            {
                contact.ContactNormal = normal;
                contact.Penetration = length - Length;
            }
            else
            {
                contact.ContactNormal = normal * -1;
                contact.Penetration = Length - length;
            }

            contact.Restitution = 0.0f;

            return 1;
        }
    }

    public class ParticleCableAnchor : ParticleContactGenerator
    {
        public Particle[] particle;

        public double MaxLength { get; set; }

        public Vector3 Anchor { get; set; }

        public double Restitution { get; set; }

        protected double CurrentLength()
        {
            Vector3 relativePos = particle[0].GetPosition() - Anchor;
            return relativePos.Magnitude;
        }

        public override int AddContact(ParticleContact contact, int limit)
        {
            double length = CurrentLength();

            if (length < MaxLength)
            {
                return 0;
            }

            contact.particle[0] = particle[0];
            contact.particle[1] = null;

            Vector3 normal = Anchor - particle[0].GetPosition();
            normal.Normalize();
            contact.ContactNormal = normal;

            contact.Penetration = length - MaxLength;
            contact.Restitution = Restitution;

            return 1;
        }
    }
}
