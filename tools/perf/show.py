import json,sys
HT=float(__import__('os').environ.get('HT','60'))
def load(p):
    return [json.loads(l) for l in open(p) if l.strip()]
def show(p, names=None):
    for r in load(p):
        if r['type']=='meta': print('META',r['gl'].get('renderer'),r.get('screen'),r.get('screen_hz'),'hz'); continue
        if r['type']=='error': print('ERROR',r); continue
        if r['type']=='hitch' and r['iter_ms']>HT: print('  HITCH t=%.0f iter=%.1f'%(r['t_ms'],r['iter_ms']), [(x['ev'],round(x['ms'],1)) for x in r['top'][:3]]); continue
        if r['type']=='mark': print('  mark %.0f %s'%(r['t_ms'],r['what'])); continue
        if r['type']!='phase': continue
        print('PHASE %s dur=%.1f main=%.0f%% proc=%.0f%% rss=%.0fMB'%(r['phase'],r['dur_s'],r['main_cpu_pct'],r['proc_cpu_pct'],r['rss_mb']))
        print('   threads',[(t['comm'],t['tid'],round(t['pct'])) for t in r['threads']])
        for k,v in sorted(r['series'].items()):
            if names and not any(n in k for n in names): continue
            print('   %-34s n=%5d mean=%7.3f p50=%7.3f p95=%7.3f p99=%7.3f max=%8.3f >33=%d >50=%d'%(k,v['n'],v['mean'],v['p50'],v['p95'],v['p99'],v['max'],v['gt33'],v['gt50']))
        print('   events',[(e['ev'],round(e['ms_per_s'],1)) for e in r['events_top'][:6]])
if __name__=='__main__': show(sys.argv[1], sys.argv[2:] or None)
