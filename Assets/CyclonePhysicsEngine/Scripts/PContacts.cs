using System;
using Cyclone.Math;

namespace Cyclone
{
    public class ParticleContact
    {
        public Particle[] particle;

        public Vector3[] ParticleMovement;

        public ParticleContact()
        {
            particle = new Particle[2];
            ParticleMovement = new Vector3[2];
            ParticleMovement[0] = new Vector3();
            ParticleMovement[1] = new Vector3();
        }

        public double Restitution { get; set; }

        public Vector3 ContactNormal { get; set; }

        public double Penetration { get; set; }

        public void Resolve(double duration)
        {
            ResolveVelocity(duration);
            ResolveInterpenetration(duration);
        }

        public double CalculateSeparatingVelocity()
        {
            Vector3 relativeVelocity = particle[0].GetVelocity();
            if (particle[1] != null)
            {
                relativeVelocity -= particle[1].GetVelocity();
            }

            return relativeVelocity * ContactNormal;
        }

        private void ResolveVelocity(double duration)
        {
            double separatingVelocity = CalculateSeparatingVelocity();

            if(separatingVelocity > 0)
            {
                return;
            }

            double newSepVelocity = -separatingVelocity * Restitution;

            Vector3 accCausedVelocity = particle[0].GetAcceleration();

            if(particle[1] != null)
            {
                accCausedVelocity -= particle[1].GetAcceleration();
            }

            double accCausedSepVelocity = accCausedVelocity * ContactNormal * duration;

            if(accCausedSepVelocity < 0)
            {
                newSepVelocity += Restitution * accCausedSepVelocity;

                if(newSepVelocity < 0)
                {
                    newSepVelocity = 0;
                }
            }

            double deltaVelocity = newSepVelocity - separatingVelocity;

            double totalInverseMass = particle[0].InverseMass;

            if(particle[1] != null)
            {
                totalInverseMass += particle[1].InverseMass;
            }

            if(totalInverseMass <= 0)
            {
                return;
            }

            double impulse = deltaVelocity / totalInverseMass;

            Vector3 impulsePerIMass = ContactNormal * impulse;
            
            particle[0].Velocity += impulsePerIMass * particle[0].InverseMass;

            if(particle[1] !=null)
            {
                particle[1].Velocity += impulsePerIMass * -particle[1].InverseMass;
            }
        }

        private void ResolveInterpenetration(double duration)
        {
            if(Penetration <= 0)
            {
                return;
            }

            double totalInverseMass = particle[0].InverseMass;

            if(particle[1] != null)
            {
                totalInverseMass += particle[1].InverseMass;
            }

            if(totalInverseMass <= 0)
            {
                return;
            }

            Vector3 movePerIMass = ContactNormal * (Penetration / totalInverseMass);

            ParticleMovement[0] = movePerIMass * particle[0].InverseMass;

            if(particle[1] != null)
            {
                ParticleMovement[1] = movePerIMass * -particle[1].InverseMass;
            }

            particle[0].Position += ParticleMovement[0];

            if(particle[1] != null)
            {
                particle[1].Position += ParticleMovement[1];
            }
        }
    }

    public class ParticleContactResolver
    {
        public int Iterations { get; set; }

        protected int IterationsUsed { get; set; }

        public ParticleContactResolver(int iterations)
        {
            Iterations = iterations;
        }

        public void ResolveContacts(ParticleContact[] contactArray, int numberOfContacts, double duration)
        {
            IterationsUsed = 0;

            while(IterationsUsed < Iterations)
            {
                double max = Double.MaxValue;
                int maxIndex = numberOfContacts;

                for(int i = 0; i < numberOfContacts; ++i)
                {
                    double sepVelocity = contactArray[i].CalculateSeparatingVelocity();

                    if((sepVelocity < max) && (sepVelocity < 0 || contactArray[i].Penetration > 0))
                    {
                        max = sepVelocity;
                        maxIndex = i;
                    }
                }

                if(maxIndex == numberOfContacts)
                {
                    break;
                }

                contactArray[maxIndex].Resolve(duration);

                Vector3[] move = contactArray[maxIndex].ParticleMovement;
                for (int i = 0; i < numberOfContacts; i++)
                {
                    if (contactArray[i].particle[0] == contactArray[maxIndex].particle[0])
                    {
                        contactArray[i].Penetration -= move[0] * contactArray[i].ContactNormal;
                    }
                    else if (contactArray[i].particle[0] == contactArray[maxIndex].particle[1])
                    {
                        contactArray[i].Penetration -= move[1] * contactArray[i].ContactNormal;
                    }
                    if (contactArray[i].particle[1] != null)
                    {
                        if (contactArray[i].particle[1] == contactArray[maxIndex].particle[0])
                        {
                            contactArray[i].Penetration += move[0] * contactArray[i].ContactNormal;
                        }
                        else if (contactArray[i].particle[1] == contactArray[maxIndex].particle[1])
                        {
                            contactArray[i].Penetration += move[1] * contactArray[i].ContactNormal;
                        }
                    }
                }

                ++IterationsUsed;
            }

        }
    }
}
