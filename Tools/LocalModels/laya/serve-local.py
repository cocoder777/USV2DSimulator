"""Official Laya HTTP API with strict no-truncation and CUDA-only deployment guards."""
import json,pathlib
import torch
import uvicorn
from laya.agent import Agent
from laya.common import build_head,encode_text,render_options
from laya.router import Router
from laya.serve import create_app
ROOT=pathlib.Path(__file__).resolve().parent
MANIFEST=json.loads((ROOT/'models.json').read_text(encoding='utf-8'))

class StrictBudgetAgent(Agent):
    def _encode_state(self,state,ids,internal,max_len=None,head_max_len=None):
        items=super()._encode_state(state,ids,internal,max_len=max_len,head_max_len=head_max_len)
        head_limit=self.cfg.get('head_max_len',192) if head_max_len is None else head_max_len
        evidence=[]
        for qid,item in zip(ids,items):
            q=internal[qid]
            head,markers,stats=build_head(self.tok,q,head_limit,option_order=q.get('option_order'))
            expected_instruction=encode_text(self.tok,f"{q['t']} question: {str(q['ins']).replace(self.tok.mask_token,' ')}",add_special_tokens=False)['input_ids']
            if not markers or head[1:markers[0]-1]!=expected_instruction:
                raise ValueError(f'{qid}: instruction would be truncated; shorten the request')
            options=render_options(q)
            order=q.get('option_order') or list(range(len(options)))
            if len(item['markers'])!=len(options) or stats['options_distinct']!=len(options):
                raise ValueError(f'{qid}: option markers or distinct options were lost')
            for slot,index in enumerate(order):
                full=encode_text(self.tok,' '+options[index].replace(self.tok.mask_token,' '),add_special_tokens=False)['input_ids']
                end=markers[slot+1] if slot+1<len(markers) else len(head)-1
                if head[markers[slot]+1:end]!=full:
                    raise ValueError(f'{qid}: option {index} would be truncated; shorten descriptions')
            if item['state_stats']['truncated']:
                raise ValueError(f'{qid}: state would be truncated; shorten the request')
            evidence.append({'question':qid,'options':len(options),'sequence_tokens':len(item['ids']),'state_tokens':item['state_stats']['state_tokens'],'instruction_tokens':len(expected_instruction),'max_len':max_len,'head_max_len':head_limit})
        self.last_budget=evidence
        print(json.dumps({'event':'budget_verified','questions':evidence}),flush=True)
        return items

class SingleModelRouter(Router):
    def predict(self,state,questions,model=None,**kwargs):
        if model not in (None,'multilingual'):
            raise ValueError('This service only serves the pinned multilingual checkpoint')
        result=super().predict(state,questions,model='multilingual',**kwargs)
        result['usage']['budget_verified']=True
        result['usage']['budget']=agent.last_budget
        result['deployment']={'model_id':MANIFEST['model_id'],'revision':MANIFEST['revision'],'device':str(agent.device)}
        return result

def require_gpu(module,args):
    if next(module.parameters()).device.type!='cuda':
        raise RuntimeError('This deployment requires CUDA; CPU inference is disabled')

if not torch.cuda.is_available(): raise RuntimeError('CUDA unavailable; refusing CPU fallback')
agent=StrictBudgetAgent(MANIFEST['path'],device='cuda',backend='eager',expected_sha256={k:v['sha256'] for k,v in MANIFEST['files'].items() if k in ('model.safetensors','rl_agent_config.json')})
if agent.device.type!='cuda': raise RuntimeError('Model did not load on CUDA')
agent.model.register_forward_pre_hook(require_gpu)
agent.cfg.update(max_len=4096,head_max_len=3072)
router=SingleModelRouter(models={'multilingual':MANIFEST['path']},device='cuda',max_loaded=1)
router.attach('multilingual',agent)
app=create_app(router)
@app.get('/deployment')
def deployment():
    return {**{k:MANIFEST[k] for k in ('model_id','revision','source_commit','path')},'device':str(agent.device),'dtype':str(agent.dtype),'backend':agent.backend,'cuda':torch.version.cuda,'torch':torch.__version__,'max_len':4096,'head_max_len':3072,'cpu_fallback_count':agent.cpu_fallback_count,'budget_guard':True}
if __name__=='__main__':uvicorn.run(app,host='127.0.0.1',port=8010)
