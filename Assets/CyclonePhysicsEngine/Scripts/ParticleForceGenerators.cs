using System;
using Cyclone.Math;

namespace Cyclone
{
    public class ParticleGravity : IParticleForceGenerator
    {
        private Vector3 gravity;

        public ParticleGravity(Vector3 gravity)
        {
            this.gravity = gravity;
        }

        public void UpdateForce(Particle particle, double duration)
        {
            if (!particle.HasFiniteMass())
            {
                return;
            }

            particle.AddForce(gravity * particle.Mass);
        }
    }

    public class ParticleDrag : IParticleForceGenerator
    {
        private double k1;

        private double k2;

        public ParticleDrag(double k1, double k2)
        {
            this.k1 = k1;
            this.k2 = k2;
        }

        public void UpdateForce(Particle particle, double duration)
        {
            Vector3 force = particle.Velocity;

            double dragCoeff = force.Magnitude;
            dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

            force.Normalize();
            force *= -dragCoeff;
            particle.AddForce(force);
        }
    }

    public class ParticleSpring : IParticleForceGenerator
    {
        private Particle other;

        private double springConstant;

        private double restLength;

        public ParticleSpring(Particle other, double springConstant, double restLength)
        {
            if (other == null)
            {
                throw new ArgumentNullException("other");
            }

            this.other = other;
            this.springConstant = springConstant;
            this.restLength = restLength;
        }

        public void UpdateForce(Particle particle, double duration)
        {
            Vector3 force = particle.Position;
            force -= other.Position;

            double magnitude = force.Magnitude;
            magnitude -= restLength;
            magnitude *= springConstant;

            force.Normalize();
            force *= -magnitude;
            particle.AddForce(force);
        }
    }

    public class ParticleAnchoredSpring : IParticleForceGenerator
    {
        private Vector3 anchor;

        private double springConstant;

        private double restLength;

        public ParticleAnchoredSpring(Vector3 anchor, double springConstant, double restLength)
        {
            this.anchor = anchor;
            this.springConstant = springConstant;
            this.restLength = restLength;
        }

        public void UpdateForce(Particle particle, double duration)
        {
            Vector3 force = particle.Position;
            force -= anchor;

            double magnitude = force.Magnitude;
            magnitude = (restLength - magnitude)*springConstant;

            force.Normalize();
            force *= magnitude;

            particle.AddForce(force);
        }
    }

    public class ParticleBungee : IParticleForceGenerator
    {
        private Particle other;

        private double springConstant;

        private double restLength;

        public ParticleBungee(Particle other, double springConstant, double restLength)
        {
            if (other == null)
            {
                throw new ArgumentNullException("other");
            }

            this.other = other;
            this.springConstant = springConstant;
            this.restLength = restLength;
        }

        public void UpdateForce(Particle particle, double duration)
        {
            Vector3 force = particle.Position;
            force -= other.Position;

            double magnitude = force.Magnitude;
            if (magnitude <= restLength)
            {
                return;
            }

            magnitude = (restLength - magnitude) * springConstant;

            force.Normalize();
            force *= magnitude;
            particle.AddForce(force);
        }
    }

    public class ParticleBuoyancy : IParticleForceGenerator
    {
        private double maxDepth;
        private double volume;
        private double waterHeight;
        private double liquidDensity;

        public ParticleBuoyancy(double maxDepth, double volume, double waterHeight, double liquidDensity = 1000.0f)
        {
            this.maxDepth = maxDepth;
            this.volume = volume;
            this.waterHeight = waterHeight;
            this.liquidDensity = liquidDensity;
        }

        public void UpdateForce(Particle particle, double duration)
        {
            double depth = particle.Position.y;

            if (depth >= waterHeight + maxDepth)
            {
                return;
            }

            Vector3 force = new Vector3();

            if (depth <= waterHeight - maxDepth)
            {
                force.y = liquidDensity*volume;
                particle.AddForce(force);
                return;
            }

            force.y = liquidDensity*volume*(depth - maxDepth - waterHeight)/2*maxDepth;
            particle.AddForce(force);
        }
    }
}
