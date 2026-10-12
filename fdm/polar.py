import numpy as np, csv, sys
from static import Static
LOADS={470:(197,45),570:(117,346)}     # front/rear lb giving ~13 in CG
def run(aero, verbose=True, masses=(470,570)):
    rows=list(csv.DictReader(open('polar_fm.csv')))
    out={}
    for m in masses:
        fr,re=LOADS[m]; st=Static(aero,front=fr,rear=re,h_ft=0.0)
        g=(4,-0.3,-2); errs=[]; res=[]
        for r in rows:
            if int(r['mass_kg'])!=m: continue
            v=float(r['speed_kmh']); t=st.trim_glide(v/3.6,guess=g)
            if t['ok']: g=(t['alpha'],t['elev'],t['gamma'])
            e=t['sink']-float(r['sink_m_s']); errs.append(e); res.append((v,t['sink'],float(r['sink_m_s']),t['alpha'],t['elev'],t['ok']))
        rms=float(np.sqrt(np.mean(np.square(errs))))
        out[m]=(rms,res,st.cg_in(),st.W)
        if verbose:
            print('m %d kg (W %.0f lb, CG %.2f in): rms %.3f m/s'%(m,st.W,st.cg_in(),rms))
            for v,s,sr,a,e,ok in res: print('   %5.0f km/h sink %.3f (FM %.2f) %+5.1f%%  alpha %5.2f elev %+.3f %s'%(v,s,sr,100*(s/sr-1),a,e,'' if ok else 'NOCONV'))
    return out
if __name__=='__main__': run(sys.argv[1])
