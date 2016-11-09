/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package acrobot_types;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class lcmt_acrobot_u implements lcm.lcm.LCMEncodable
{
    public long timestamp;
    public double tau;
 
    public lcmt_acrobot_u()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x5722ea20a6b338b8L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(acrobot_types.lcmt_acrobot_u.class))
            return 0L;
 
        classes.add(acrobot_types.lcmt_acrobot_u.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeLong(this.timestamp); 
 
        outs.writeDouble(this.tau); 
 
    }
 
    public lcmt_acrobot_u(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public lcmt_acrobot_u(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static acrobot_types.lcmt_acrobot_u _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        acrobot_types.lcmt_acrobot_u o = new acrobot_types.lcmt_acrobot_u();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.timestamp = ins.readLong();
 
        this.tau = ins.readDouble();
 
    }
 
    public acrobot_types.lcmt_acrobot_u copy()
    {
        acrobot_types.lcmt_acrobot_u outobj = new acrobot_types.lcmt_acrobot_u();
        outobj.timestamp = this.timestamp;
 
        outobj.tau = this.tau;
 
        return outobj;
    }
 
}
