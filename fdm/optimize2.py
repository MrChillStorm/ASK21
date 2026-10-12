"""stage 2: local CMA-ES from the stage-1 best, spin rate and height loss per turn weighted up."""
import numpy as np, json, sys, time, cma
import objective, optimize as O
objective.W_RATE, objective.W_FT, objective.W_KIAS = 3.0, 3.0, 0.3
if __name__ == '__main__':
    seed = int(sys.argv[1]); gens = int(sys.argv[2]); start = json.load(open(sys.argv[3]))['P']
    x0 = np.array([start[n] for n in O.NAMES]); u0 = (x0 - O.LO) / (O.HI - O.LO)
    es = cma.CMAEvolutionStrategy(u0, 0.12, {'bounds': [0, 1], 'popsize': 12, 'seed': seed, 'verbose': -9})
    best = (1e9, None); log = open('opt_log_%d.txt' % seed, 'a')
    # include the start point itself in the first generation
    for g in range(gens):
        U = es.ask()
        if g == 0: U[0] = np.clip(u0, 0, 1)
        L = []
        for k, u in enumerate(U):
            P = O.to_P(u)
            try: l, info = objective.evaluate(P, tag='s%d_%d_%d' % (seed, g, k))
            except Exception as e: l, info = 50.0, {'notes': [str(e)]}
            L.append(l); ch = info.get('ch')
            log.write(json.dumps(dict(g=g, k=k, loss=l, P=P, notes=info.get('notes'), ch=ch)) + '\n'); log.flush()
            if l < best[0]:
                best = (l, P); json.dump(dict(loss=l, P=P, notes=info.get('notes'), ch=ch), open('opt_best_%d.json' % seed, 'w'), indent=1)
        es.tell(U, L)
        print('gen %d  min %.3f  median %.3f  best %.3f  %s' % (g, min(L), float(np.median(L)), best[0], time.strftime('%H:%M:%S')), flush=True)
