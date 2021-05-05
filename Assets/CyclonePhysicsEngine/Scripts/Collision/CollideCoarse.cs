using System;
using System.Text;

namespace Cyclone
{
    public class PotentialContact
    {
        public RigidBody[] body = new RigidBody[2];
    }

    public abstract class BoundingVolume
    {
        public abstract double Size { get; }

        public abstract bool Overlaps(BoundingVolume other);

        public abstract double GetGrowth(BoundingVolume other);
    }

    public class BVHNode
    {
        public BVHNode parent;
        public BVHNode[] children = new BVHNode[2];
        public BoundingSphere volume;
        public RigidBody body;

        public BVHNode(BVHNode parent, BoundingSphere volume, RigidBody body = null)
        {
            this.parent = parent;
            this.volume = volume;
            this.body = body;
        }

        public void Remove()
        {
            if (parent != null)
            {
                BVHNode sibling;
                if (parent.children[0] == this)
                {
                    sibling = parent.children[1];
                }
                else
                {
                    sibling = parent.children[0];
                }

                parent.volume = sibling.volume;
                parent.body = sibling.body;
                parent.children[0] = sibling.children[0];
                parent.children[1] = sibling.children[1];

                sibling.parent = null;
                sibling.body = null;
                sibling.children[0] = null;
                sibling.children[1] = null;
                sibling = null;

                parent.RecalculateBoundingVolume();
            }

            if (children[0] != null)
            {
                children[0].parent = null;
                children[0] = null;
            }
            if (children[1] != null)
            {
                children[1].parent = null;
                children[1] = null;
            }
        }

        public bool IsLeaf()
        {
            return body != null;
        }

        public uint GetPotentialContacts(PotentialContact[] contacts, uint limit)
        {
            if (IsLeaf() || limit == 0)
            {
                return 0;
            }

            uint index = 0;

            return children[0].GetPotentialContactsWith(children[1], contacts, index, limit);
        }

        public uint GetPotentialContactsWith(BVHNode other, PotentialContact[] contacts, uint index, uint limit)
        {
            if (!Overlaps(other) || limit == 0)
            {
                return 0;
            }

            if (IsLeaf() && other.IsLeaf())
            {
                contacts[index].body[0] = body;
                contacts[index].body[1] = other.body;
                return 1;
            }

            if (other.IsLeaf() || (!IsLeaf() && volume.Size >= other.volume.Size))
            {
                uint count = children[0].GetPotentialContactsWith(other, contacts, index, limit);

                if (limit > count)
                {
                    return count + children[1].GetPotentialContactsWith(other, contacts, index + count, limit - count);
                }
                else
                {
                    return count;
                }
            }
            else
            {
                uint count = GetPotentialContactsWith(other.children[0], contacts, index, limit);

                if (limit > count)
                {
                    return count + GetPotentialContactsWith(other.children[1], contacts, index + count, limit - count);
                }
                else
                {
                    return count;
                }
            }
        }

        public bool Overlaps(BVHNode other)
        {
            return volume.Overlaps(other.volume);
        }

        public void Insert(RigidBody newBody, BoundingSphere newVolume)
        {
            if (IsLeaf())
            {
                children[0] = new BVHNode(this, volume, body);

                children[1] = new BVHNode(this, newVolume, newBody);

                this.body = null;

                RecalculateBoundingVolume();
            }

            else
            {
                if (children[0].volume.GetGrowth(newVolume) <
                    children[1].volume.GetGrowth(newVolume))
                {
                    children[0].Insert(newBody, newVolume);
                }
                else
                {
                    children[1].Insert(newBody, newVolume);
                }
            }
        }

        public void RecalculateBoundingVolume(bool recurse = true)
        {
            if (IsLeaf())
            {
                return;
            }

            volume = new BoundingSphere(children[0].volume, children[1].volume);

            if (parent != null)
            {
                parent.RecalculateBoundingVolume(true);
            }
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            sb.AppendFormat("{0}", volume);
            return sb.ToString();
        }
    }
}
