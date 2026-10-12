import numpy as np, json, sys, os, cma, time
import objective
NAMES=['CN90_wing','stall_drop','boff','dstall','w_ca','xcp90','CN90_ht','CN90_vt','wake_k','wake_kv','fin_blank','cdc','kpot_aft','wing_clmax_k','a_peak_root','tip_clmax_k','a_peak_tip','flap_eff']
LO=np.array([1.1,0.03,3.0, 8.0,4.0,0.42,0.9,0.9,0.3,0.0,0.25,0.6,0.2,0.80,12.0,0.85,15.0,0.85])
HI=np.array([1.9,0.35,12.0,25.0,18.0,0.56,1.5,1.4,1.8,1.0,0.45,1.3,1.0,0.90,17.0,1.00,19.0,1.05])
X0=np.array([1.3,0.10,8.0,15.0,14.0,0.50,1.2,1.15,0.5,0.6,0.35,1.0,0.5,0.85,15.0,0.92,17.5,0.95])
def to_P(u):
    x=LO+(HI-LO)*np.clip(u,0,1); return {n:float(v) for n,v in zip(NAMES,x)}
if __name__=='__main__':
    seed=int(sys.argv[1]) if len(sys.argv)>1 else 1
    gens=int(sys.argv[2]) if len(sys.argv)>2 else 25
    u0=(X0-LO)/(HI-LO)
    es=cma.CMAEvolutionStrategy(u0,0.2,{'bounds':[0,1],'popsize':12,'seed':seed,'verbose':-9})
    best=(1e9,None); log=open('opt_log_%d.txt'%seed,'a')
    for g in range(gens):
        U=es.ask(); L=[]
        for k,u in enumerate(U):
            P=to_P(u)
            try: l,info=objective.evaluate(P,tag='g%d_%d'%(g,k))
            except Exception as e: l,info=50.0,{'notes':[str(e)]}
            L.append(l)
            ch=info.get('ch')
            log.write(json.dumps(dict(g=g,k=k,loss=l,P=P,notes=info.get('notes'),ch=ch))+'\n'); log.flush()
            if l<best[0]:
                best=(l,P); json.dump(dict(loss=l,P=P,notes=info.get('notes'),ch=ch),open('opt_best_%d.json'%seed,'w'),indent=1)
        es.tell(U,L)
        print('gen %d  min %.3f  median %.3f  best %.3f  %s'%(g,min(L),float(np.median(L)),best[0],time.strftime('%H:%M:%S')),flush=True)
