"""Adversarial CPU-only session admission, source custody and cleanup tests."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim_admission import (AdmissionError, Clock, active_session, admit, atomic_json, digest, ledger_event,
                           read_ledger, reconcile, remaining_execution, session_accounting, snapshot_tree, snapshot_public, private_input)
from sim_watchdog import cleanup_container, matches_container, process_identity


def authorization():
    a = {'kind': 'explicit_user_authorized_overnight_simulation_session', 'session_id': 'session',
         'hardware_authorized': False, 'boot_id': 'boot', 'start_unix': 100000., 'start_monotonic': 100.,
         'experimental_deadline_unix': 127000., 'final_deadline_unix': 128800.,
         'limits': {'default_diagnostic_seconds': 300, 'maximum_named_batch_seconds': 1800,
                    'maximum_training_job_seconds': 3600, 'maximum_total_training_allocation_seconds': 7200,
                    'concurrent_gpu_jobs': 1}}
    for label, key in [('start_utc', 'start_unix'), ('experimental_deadline_utc', 'experimental_deadline_unix'), ('final_deadline_utc', 'final_deadline_unix')]:
        a[label] = datetime.fromtimestamp(a[key], timezone.utc).isoformat()
    return a


def admission(a=None, events=(), **changes):
    kwargs = dict(category='diagnostic', name='', justification='', seconds=300, cleanup_seconds=30,
                  training_prerequisites=None, clock=Clock(100100., 200., 'boot'), authorization_sha256='auth')
    kwargs.update(changes)
    return admit(a or authorization(), events, **kwargs)


class AdmissionTests(unittest.TestCase):
    def test_default_allocation_includes_cleanup(self):
        job = admission()
        self.assertEqual(job['stop_deadline_monotonic'], 470)
        self.assertEqual(job['hard_deadline_monotonic'], 500)
        self.assertEqual(remaining_execution(job, Clock(100101, 201, 'boot')), 269)

    def test_extended_categories_require_name_justification(self):
        for category in ['asset-build', 'initialization', 'integration', 'evaluation']:
            with self.assertRaises(AdmissionError): admission(category=category, seconds=1800)
            self.assertEqual(admission(category=category, seconds=1800, name='batch', justification='measured initialization need')['allocation_seconds'], 1800)
            with self.assertRaises(AdmissionError): admission(category=category, seconds=1801, name='batch', justification='reason')
        with self.assertRaises(AdmissionError): admission(seconds=301)
        with self.assertRaises(AdmissionError): admission(category='foundation-finetuning')

    def test_clock_reboot_rollback_and_utc_deadline(self):
        for clock in [Clock(100100, 200, 'newboot'), Clock(99999, 200, 'boot'), Clock(100100, 99, 'boot'),
                      Clock(126800, 200, 'boot'), Clock(100100, 26900, 'boot')]:
            with self.assertRaises(AdmissionError): admission(clock=clock)
        self.assertEqual(admission(clock=Clock(126700, 26800, 'boot'))['allocation_seconds'], 300)
        with self.assertRaises(AdmissionError): admission(cleanup_seconds=5)

    def test_restart_does_not_reset_clock_or_allowance(self):
        event = dict(admission(), event='ADMITTED', run_id='first')
        finished = dict(event, event='FINISHED', utc=100500., monotonic=600., cleanup_verified=True)
        with self.assertRaises(AdmissionError): admission(events=[event])
        with self.assertRaises(AdmissionError): admission(events=[event, finished], clock=Clock(100499, 601, 'boot'))
        with self.assertRaises(AdmissionError): admission(events=[event, finished], clock=Clock(100501, 599, 'boot'))
        with self.assertRaises(AdmissionError): admission(events=[event, finished], clock=Clock(100501, 601, 'boot'), authorization_sha256='changed')
        self.assertEqual(admission(events=[event, finished], clock=Clock(100501, 601, 'boot'))['hard_deadline_monotonic'], 901)

    def test_cleanup_failure_blocks_next_job(self):
        event = dict(admission(), event='ADMITTED', run_id='first')
        for end in [dict(event, event='FINISHED'), dict(event, event='FINISHED', cleanup_verified=False)]:
            with self.assertRaises(AdmissionError): admission(events=[event, end])
        good = dict(event, event='FINISHED', cleanup_verified=True)
        bad = dict(event, event='FINISHED', cleanup_verified=False)
        with self.assertRaises(AdmissionError): admission(events=[event, good, bad])

    def test_training_allocations_count_failures_and_require_evidence(self):
        prereq = {k: 'recorded evidence' for k in ['working_environment', 'controller_gap', 'evaluation_criterion', 'checkpoint_plan']}
        kwargs = dict(category='training', seconds=3600, name='pilot', justification='measured gap', training_prerequisites=prereq)
        with self.assertRaises(AdmissionError): admission(**dict(kwargs, training_prerequisites={}))
        first = dict(admission(**kwargs), run_id='a', event='ADMITTED')
        second = dict(first, run_id='b')
        events = [first, dict(first, event='FINISHED', status='FAIL', cleanup_verified=True), second, dict(second, event='FINISHED', status='FAIL', cleanup_verified=True)]
        with self.assertRaises(AdmissionError): admission(events=events, **kwargs)
        old = dict(first, session_id='previous-checkpoint', allocation_seconds=999999)
        self.assertEqual(admission(events=[old], **kwargs)['allocation_seconds'], 3600)

    def test_cumulative_session_allocation_cap_counts_failed_and_stopped_jobs(self):
        a = authorization(); a['limits']['maximum_session_allocated_seconds'] = 1000
        first = dict(admission(a, seconds=300), run_id='a', event='ADMITTED')
        done = [first, dict(first, event='FINISHED', status='WATCHDOG_STOPPED', cleanup_verified=True)]
        second = dict(admission(a, events=done, seconds=300), run_id='b', event='ADMITTED')
        done += [second, dict(second, event='FINISHED', status='FAIL', cleanup_verified=True)]
        third = dict(admission(a, events=done, seconds=300), run_id='c', event='ADMITTED')
        done += [third, dict(third, event='FINISHED', status='PASS', cleanup_verified=True)]
        with self.assertRaisesRegex(AdmissionError, 'Cumulative session allocation'):
            admission(a, events=done, seconds=300)          # 900 + 300 > 1000, no refunds
        self.assertEqual(admission(a, events=done, seconds=100)['allocation_seconds'], 100)
        other = [dict(e, session_id='previous-session') for e in done]
        self.assertEqual(admission(a, events=other, seconds=300)['allocation_seconds'], 300)  # other sessions never charged here
        a['limits']['maximum_session_allocated_seconds'] = float('nan')
        with self.assertRaises(AdmissionError): admission(a, seconds=300)

    def test_nonfinite_and_expanded_authorization_refused(self):
        for value in [float('nan'), float('inf'), True]:
            with self.assertRaises(AdmissionError): admission(seconds=value)
        a=authorization(); a['limits']['maximum_named_batch_seconds']=2000
        with self.assertRaises(AdmissionError): admission(a)
        a=authorization(); a['final_deadline_unix'] += 1
        with self.assertRaises(AdmissionError): admission(a)

    def test_deadline_fails_closed_on_runtime_clock_changes(self):
        job=admission()
        for clock in [Clock(100101, 201, 'other'), Clock(100099, 201, 'boot'), Clock(100101, 199, 'boot'), Clock(100370, 201, 'boot'), Clock(100101, 470, 'boot')]:
            self.assertEqual(remaining_execution(job, clock), 0)

    def test_durable_ledger_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'ledger.jsonl'
            ledger_event(path, {'event':'ADMITTED','run_id':'a'})
            ledger_event(path, {'event':'FINISHED','run_id':'a'})
            self.assertEqual(len(read_ledger(path)),2)


class SourceCustodyTests(unittest.TestCase):
    def test_snapshot_survives_live_edit_without_hardlinks(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); source=root/'source'; source.mkdir(); (source/'code.py').write_text('original')
            hashes=snapshot_tree(source,root/'snapshot')
            (source/'code.py').write_text('next experiment')
            self.assertEqual((root/'snapshot/code.py').read_text(),'original')
            self.assertEqual(digest(root/'snapshot/code.py'),hashes['code.py'])
            self.assertNotEqual((source/'code.py').stat().st_ino,(root/'snapshot/code.py').stat().st_ino)

    def test_source_mutation_during_copy_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); source=root/'source'; source.mkdir(); (source/'a').write_text('before')
            original=shutil.copyfile
            def racing(src,dst):
                result=original(src,dst); Path(src).write_text('after'); return result
            with patch('sim_admission.shutil.copyfile',side_effect=racing), self.assertRaises(AdmissionError):
                snapshot_tree(source,root/'snapshot')

    def test_symlink_escape_and_special_files_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); source=root/'source'; source.mkdir(); (source/'escape').symlink_to('/etc/os-release')
            with self.assertRaises(AdmissionError): snapshot_tree(source,root/'snapshot')
            (source/'escape').unlink(); os.mkfifo(source/'pipe')
            with self.assertRaises(AdmissionError): snapshot_tree(source,root/'other')

    def test_private_mount_requires_explicit_private_root(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); (root/'public').mkdir(); (root/'private').mkdir(); (root/'deps').mkdir()
            workspace={'repositories':[{'worktree':str(root/'public'),'visibility':'public'},{'worktree':str(root/'private'),'visibility':'private'}]}
            self.assertEqual(private_input('private-driver='+str(root/'private'),workspace)[0],'private-driver')
            for spec in ['../escape='+str(root/'private'),'source='+str(root/'public'),'deps='+str(root/'deps')]:
                with self.assertRaises(AdmissionError): private_input(spec,workspace)
            workspace['private_input_roots']=[str(root/'deps')]
            self.assertEqual(private_input('deps='+str(root/'deps'),workspace)[1],root/'deps')

    def test_public_multidirectory_capture_detects_late_isaac_edit(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); repo=root/'repo'; repo.mkdir(); (repo/'isaac').mkdir(); (repo/'tools').mkdir()
            (repo/'isaac/a').write_text('a'); (repo/'tools/b').write_text('b')
            subprocess.run(['git','init','-q',str(repo)],check=True)
            original=shutil.copyfile
            def racing(src,dst):
                result=original(src,dst)
                if str(src).endswith('/tools/b'): (repo/'isaac/a').write_text('changed later')
                return result
            with patch('sim_admission.shutil.copyfile',side_effect=racing), self.assertRaises(AdmissionError):
                snapshot_public(repo,root/'snapshot')


class WatchdogTests(unittest.TestCase):
    def test_cleanup_requires_exact_id_and_two_labels(self):
        job={'container_id':'a'*64,'run_id':'panthera_sim_a','run_token':'token','session_id':'session'}
        obj={'Id':'a'*64,'Name':'/panthera_sim_a','Config':{'Labels':{'panthera.run_token':'token','panthera.session_id':'session'}}}
        self.assertTrue(matches_container(obj,job))
        for key in ['container_id','run_id','run_token','session_id']:
            wrong=dict(job);wrong[key]='wrong'
            calls=[]
            def runner(argv,**kwargs):
                calls.append(argv);return subprocess.CompletedProcess(argv,0,json.dumps([obj]),'')
            self.assertTrue(cleanup_container(wrong,runner))
            self.assertFalse(any(a[1] in ['stop','rm'] for a in calls))

    def test_matching_cleanup_stops_only_exact_id(self):
        job={'container_id':'a'*64,'run_id':'panthera_sim_a','run_token':'token','session_id':'session'}
        obj={'Id':'a'*64,'Name':'/panthera_sim_a','Config':{'Labels':{'panthera.run_token':'token','panthera.session_id':'session'}}}
        calls=[]
        def runner(argv,**kwargs):
            calls.append(argv);return subprocess.CompletedProcess(argv,0,json.dumps([obj]) if argv[1]=='inspect' else '','')
        self.assertEqual(cleanup_container(job,runner),[])
        self.assertEqual([a[1] for a in calls],['inspect','stop','rm'])
        self.assertTrue(all(a[-1]=='a'*64 for a in calls))

    def test_stop_timeout_still_attempts_force_removal(self):
        job={'container_id':'a'*64,'run_id':'panthera_sim_a','run_token':'token','session_id':'session'}
        obj={'Id':'a'*64,'Name':'/panthera_sim_a','Config':{'Labels':{'panthera.run_token':'token','panthera.session_id':'session'}}}
        calls=[]
        def runner(argv,**kwargs):
            calls.append(argv)
            if argv[1]=='stop': raise subprocess.TimeoutExpired(argv,8)
            return subprocess.CompletedProcess(argv,0,json.dumps([obj]) if argv[1]=='inspect' else '','')
        self.assertTrue(cleanup_container(job,runner))
        self.assertEqual([a[1] for a in calls],['inspect','stop','rm'])

    def test_independent_watchdog_survives_launcher_exit(self):
        # Real detached watchdog process; fake docker executable, no container or GPU use.
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); (root/'output').mkdir(); (root/'bin').mkdir()
            fake=root/'bin/docker'
            fake.write_text('#!'+sys.executable+'\nimport json,os,sys\nfrom pathlib import Path\np=Path(os.environ["WATCHDOG_TEST_ROOT"])\nj=json.loads((p/"job.json").read_text())\nwith (p/"calls").open("a") as f:f.write(json.dumps(sys.argv[1:])+"\\n")\nif sys.argv[1]=="inspect":print(json.dumps([{"Id":j["container_id"],"Name":"/"+j["run_id"],"Config":{"Labels":{"panthera.run_token":j["run_token"],"panthera.session_id":j["session_id"]}}}]))\n')
            fake.chmod(0o755)
            tools=Path(__file__).resolve().parents[1]
            parent=root/'parent.py'
            parent.write_text('import sys,os,json,subprocess,fcntl,time\nfrom pathlib import Path\nsys.path.insert(0,'+repr(str(tools))+')\nfrom sim_admission import Clock,atomic_json\nfrom sim_watchdog import process_identity\np=Path('+repr(str(root))+')\nc=Clock.now()\nf=(p/"global.lock").open("a");fcntl.flock(f,fcntl.LOCK_EX)\nj={"run_id":"panthera_sim_test","container_id":"a"*64,"run_token":"t","session_id":"s","authorization_sha256":"h","ledger_path":str(p/"ledger.jsonl"),"output":str(p/"output"),"parent":process_identity(os.getpid()),"boot_id":c.boot_id,"utc":c.utc,"monotonic":c.monotonic,"stop_deadline_utc":c.utc+5,"stop_deadline_monotonic":c.monotonic+5,"experimental_deadline_utc":c.utc+10,"experimental_deadline_monotonic":c.monotonic+10}\natomic_json(p/"job.json",j)\nsubprocess.Popen([sys.executable,'+repr(str(tools/'sim_watchdog.py'))+',"--job",str(p/"job.json"),"--lock-fd",str(f.fileno())],start_new_session=True,pass_fds=(f.fileno(),),stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)\nos._exit(0)\n')
            env=dict(os.environ,PATH=str(root/'bin')+os.pathsep+os.environ['PATH'],WATCHDOG_TEST_ROOT=str(root))
            subprocess.run([sys.executable,str(parent)],env=env,timeout=3,check=True)
            deadline=time.monotonic()+5
            while not (root/'ledger.jsonl').exists() and time.monotonic()<deadline: time.sleep(.05)
            receipt=json.loads((root/'watchdog.json').read_text())
            self.assertEqual(receipt['reason'],'launcher_process_disappeared_or_reused')
            self.assertEqual(json.loads((root/'output/run.json').read_text())['status'],'FAIL')
            calls=[json.loads(x) for x in (root/'calls').read_text().splitlines()]
            self.assertEqual([a[0] for a in calls],['inspect','stop','rm'])
            self.assertTrue(all(a[-1]=='a'*64 for a in calls))


class LauncherIntegrationTests(unittest.TestCase):
    def test_full_launcher_uses_snapshot_while_live_code_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); root.chmod(0o700); (root/'evidence').mkdir()
            tools=Path(__file__).resolve().parents[1]
            public=root/'public'; (public/'tools/probes').mkdir(parents=True); (public/'isaac').mkdir()
            for name in ['run_isolated_isaac.py','sim_admission.py','sim_watchdog.py']:
                shutil.copyfile(tools/name,public/'tools'/name)
            (public/'tools/probes/smoke.py').write_text('# immutable first probe\n')
            (public/'isaac/module.py').write_text('# source\n')
            repositories=[]
            for name in ['public','driver','loco','wbc']:
                path=root/name; path.mkdir(exist_ok=True)
                subprocess.run(['git','init','-q',str(path)],check=True)
                subprocess.run(['git','-C',str(path),'-c','user.name=Test','-c','user.email=test@example.invalid','commit','-q','--allow-empty','-m','fixture'],check=True)
                repositories.append({'repository':name,'worktree':str(path),'visibility':'public' if name=='public' else 'private'})
            (root/'identity.json').write_text(json.dumps({'hardware_authorized':False}))
            session=root/'sessions/session';session.mkdir(parents=True)
            now=Clock.now(); auth=authorization(); auth.update(start_unix=now.utc,start_monotonic=now.monotonic,boot_id=now.boot_id,experimental_deadline_unix=now.utc+27000,final_deadline_unix=now.utc+28800)
            for label,key in [('start_utc','start_unix'),('experimental_deadline_utc','experimental_deadline_unix'),('final_deadline_utc','final_deadline_unix')]: auth[label]=datetime.fromtimestamp(auth[key],timezone.utc).isoformat()
            atomic_json(session/'authorization.json',auth)
            atomic_json(root/'workspace-lock.json',{'repositories':repositories,'hardware_authorized':False,'identity_contract':'identity.json','session_authorization':{'path':'sessions/session/authorization.json','sha256':digest(session/'authorization.json')}})
            for name in ['kit','ov','pip','gl','compute','warp']: (root/'cache'/name).mkdir(parents=True)
            (root/'bin').mkdir()
            fake=root/'bin/docker'
            fake.write_text('#!'+sys.executable+'\n'+r'''import json,os,sys
from pathlib import Path
p=Path(os.environ['LAUNCHER_TEST_ROOT']);args=sys.argv[1:]
if args[:2]==['image','inspect']:
 print(json.dumps([{'Id':'sha256:'+'b'*64,'RepoDigests':[],'Architecture':'amd64'}]));sys.exit(0)
if args[0]=='create':
 (p/'docker-created.json').write_text(json.dumps(args));print('a'*64);sys.exit(0)
a=json.loads((p/'docker-created.json').read_text()); labels={a[i+1].split('=',1)[0]:a[i+1].split('=',1)[1] for i,x in enumerate(a) if x=='--label'}
if args[0]=='inspect':
 print(json.dumps([{'Id':'a'*64,'Name':'/'+a[a.index('--name')+1],'Config':{'Labels':labels,'User':'1234:1234'},'HostConfig':{'NetworkMode':'none','IpcMode':'private','Privileged':False,'Devices':[],'PortBindings':{},'CapAdd':[],'CapDrop':['ALL'],'SecurityOpt':['no-new-privileges']},'State':{'ExitCode':0}}]))
elif args[0]=='start':
 (p/'public/tools/probes/smoke.py').write_text('# next probe edited concurrently\n')
 out=p/'evidence/smoke';(out/'metrics.json').write_text('{}');(out/'probe.json').write_text(json.dumps({'status':'PASS','artifacts':['metrics.json']}))
''')
            fake.chmod(0o755)
            gpu=root/'bin/nvidia-smi';gpu.write_text('#!'+sys.executable+'\nprint("fake CPU test only")\n');gpu.chmod(0o755)
            env=dict(os.environ,PATH=str(root/'bin')+os.pathsep+os.environ['PATH'],LAUNCHER_TEST_ROOT=str(root))
            result=subprocess.run([sys.executable,str(public/'tools/run_isolated_isaac.py'),'tools/probes/smoke.py','--workspace-lock',str(root/'workspace-lock.json'),'--output',str(root/'evidence/smoke'),'--cache',str(root/'cache'),'--seconds','60','--probe-mode','new-moving-candidate'],env=env,capture_output=True,text=True,timeout=15)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            manifest=json.loads((root/'evidence/smoke/run.json').read_text())
            self.assertEqual(manifest['status'],'PASS')
            self.assertEqual((Path(manifest['source_snapshot'])/'tools/probes/smoke.py').read_text(),'# immutable first probe\n')
            self.assertNotIn('changed_inputs_during_run',manifest)
            self.assertTrue(read_ledger(session/'ledger.jsonl')[-1]['cleanup_verified'])
            argv=json.loads((root/'docker-created.json').read_text())
            self.assertFalse(any('source='+str(public)+',' in x for x in argv))


if __name__ == '__main__': unittest.main()


def measured_authorization(cap=27000.):
    a = authorization(); a['schema_version'] = 2
    a['limits'].update(maximum_session_measured_launcher_seconds=cap, maximum_session_allocated_seconds=None,
                       maximum_training_job_seconds=0, maximum_total_training_allocation_seconds=0)
    return a


class MeasuredRuntimeAccountingTests(unittest.TestCase):
    """Cumulative cap on MEASURED launcher wall time with reservations held until reconciliation."""

    def _run(self, events, run_id, seconds, t0, finish_after=None, status='PASS', cleanup=True, **kw):
        job = dict(admission(measured_authorization(), events, seconds=seconds, clock=Clock(100000. + t0, t0, 'boot'), **kw), run_id=run_id, event='ADMITTED')
        events.append(job)
        if finish_after is not None:
            events.append({'session_id': 'session', 'authorization_sha256': 'auth', 'run_id': run_id, 'event': 'FINISHED', 'status': status,
                           'utc': 100000. + t0 + finish_after, 'monotonic': t0 + finish_after, 'boot_id': 'boot', 'cleanup_verified': cleanup})
        return job

    def _settle(self, events, run_id, t):
        event = reconcile(measured_authorization(), events, run_id=run_id, clock=Clock(100000. + t, t, 'boot'), authorization_sha256='auth')
        events.append(event); return event

    def test_early_success_charges_measured_not_reserved(self):
        events = []
        self._run(events, 'a', 900, 200, finish_after=90, category='evaluation', name='n', justification='j')
        settled = self._settle(events, 'a', 300)
        self.assertEqual((settled['charged_seconds'], settled['reservation_seconds'], settled['released_reservation_seconds']), (90., 900., 810.))
        account = session_accounting(measured_authorization(), events)
        self.assertEqual((account['charged_completed_seconds'], account['active_reservation_seconds'], account['reserved_history_seconds']), (90., 0., 900.))

    def test_early_failure_and_timeout_charge_their_measured_runtime(self):
        events = []
        self._run(events, 'a', 300, 200, finish_after=40, status='FAIL'); self._settle(events, 'a', 250)
        self._run(events, 'b', 300, 300, finish_after=300, status='WATCHDOG_STOPPED'); self._settle(events, 'b', 610)
        account = session_accounting(measured_authorization(), events)
        self.assertEqual(account['charged_completed_seconds'], 340.); self.assertEqual(account['active_reservation_seconds'], 0.)

    def test_cap_counts_charged_plus_active_reservations(self):
        a = measured_authorization(cap=1000.); events = []
        first = dict(admission(a, events, seconds=300, clock=Clock(100200., 200., 'boot')), run_id='a', event='ADMITTED'); events.append(first)
        events.append(dict(first, event='FINISHED', status='PASS', utc=100260., monotonic=260., cleanup_verified=True))
        # finished but not reconciled: the full 300 s reservation is still active
        self.assertEqual(session_accounting(a, events)['active_reservation_seconds'], 300.)
        second = dict(admission(a, events, seconds=300, clock=Clock(100300., 300., 'boot')), run_id='b', event='ADMITTED'); events.append(second)
        events.append(dict(second, event='FINISHED', status='PASS', utc=100400., monotonic=400., cleanup_verified=True))
        events.append(reconcile(a, events, run_id='a', clock=Clock(100401., 401., 'boot'), authorization_sha256='auth'))   # charged 60
        # charged 60 + active 300 (b) + new 300 = 660 <= 1000 admits; 60 + 300 + 700 > 1000 refuses
        self.assertEqual(admission(a, events, seconds=300, clock=Clock(100402., 402., 'boot'))['allocation_seconds'], 300)
        with self.assertRaisesRegex(AdmissionError, 'measured launcher allowance'):
            admission(a, events, seconds=700, clock=Clock(100402., 402., 'boot'), category='evaluation', name='n', justification='j')
        events.append(reconcile(a, events, run_id='b', clock=Clock(100403., 403., 'boot'), authorization_sha256='auth'))   # charged 100
        self.assertEqual(admission(a, events, seconds=800, clock=Clock(100404., 404., 'boot'), category='evaluation', name='n', justification='j')['allocation_seconds'], 800)
        with self.assertRaises(AdmissionError):
            admission(a, events, seconds=841, clock=Clock(100404., 404., 'boot'), category='evaluation', name='n', justification='j')

    def test_missing_receipt_holds_reservation_and_blocks(self):
        events = []; self._run(events, 'a', 300, 200)
        with self.assertRaisesRegex(AdmissionError, 'No terminal receipt'): self._settle(events, 'a', 600)
        with self.assertRaisesRegex(AdmissionError, 'lacks a terminal receipt'): admission(measured_authorization(), events, clock=Clock(100600., 600., 'boot'))
        self.assertEqual(session_accounting(measured_authorization(), events)['active_reservation_seconds'], 300.)
        events.append({'session_id': 'session', 'authorization_sha256': 'auth', 'run_id': 'a', 'event': 'FINISHED', 'status': 'FAIL', 'utc': 100500., 'monotonic': 500., 'boot_id': 'boot', 'cleanup_verified': False})
        with self.assertRaisesRegex(AdmissionError, 'without verified cleanup'): self._settle(events, 'a', 600)

    def test_crash_restart_reconciles_from_durable_ledger_once(self):
        with tempfile.TemporaryDirectory() as tmp:
            ledger = Path(tmp) / 'ledger.jsonl'; a = measured_authorization()
            job = dict(admission(a, [], seconds=300, clock=Clock(100200., 200., 'boot')), run_id='a', event='ADMITTED'); ledger_event(ledger, job)
            ledger_event(ledger, dict(job, event='FINISHED', status='PASS', utc=100250., monotonic=250., cleanup_verified=True))
            # launcher died before reconciling; a later process reconciles from the durable ledger
            settled = reconcile(a, read_ledger(ledger), run_id='a', clock=Clock(100900., 900., 'boot'), authorization_sha256='auth'); ledger_event(ledger, settled)
            self.assertEqual(settled['charged_seconds'], 50.)
            with self.assertRaisesRegex(AdmissionError, 'already reconciled'):
                reconcile(a, read_ledger(ledger), run_id='a', clock=Clock(100901., 901., 'boot'), authorization_sha256='auth')
            ledger_event(ledger, settled)   # a duplicate line on disk is detected, never double charged silently
            with self.assertRaisesRegex(AdmissionError, 'Duplicate reconciliation'): session_accounting(a, read_ledger(ledger))

    def test_double_admission_refused_while_job_active(self):
        events = []; self._run(events, 'a', 300, 200)
        with self.assertRaises(AdmissionError): admission(measured_authorization(), events, clock=Clock(100201., 201., 'boot'))

    def test_expired_deadline_refuses_admission(self):
        a = measured_authorization()
        with self.assertRaises(AdmissionError): admission(a, [], seconds=300, clock=Clock(126800., 26800., 'boot'))
        with self.assertRaises(AdmissionError): admission(a, [], seconds=300, clock=Clock(127001., 27001., 'boot'))

    def test_negative_and_nonfinite_durations_refused(self):
        events = []; job = self._run(events, 'a', 300, 200)
        events.append(dict(job, event='FINISHED', status='PASS', utc=100100., monotonic=100., cleanup_verified=True))   # ends before it started
        with self.assertRaisesRegex(AdmissionError, 'Negative measured runtime'): self._settle(events, 'a', 600)
        events[-1] = dict(job, event='FINISHED', status='PASS', utc=float('nan'), monotonic=260., cleanup_verified=True)
        with self.assertRaises(AdmissionError): self._settle(events, 'a', 600)
        bad = dict(job, event='ADMITTED', run_id='b', allocation_seconds=float('inf'))
        with self.assertRaises(AdmissionError): session_accounting(measured_authorization(), events[:1] + [bad])

    def test_owner_mismatch_refused(self):
        events = []; self._run(events, 'a', 300, 200, finish_after=30)
        with self.assertRaisesRegex(AdmissionError, 'Owner mismatch'):
            reconcile(measured_authorization(), events, run_id='a', clock=Clock(100600., 600., 'boot'), authorization_sha256='other')
        foreign = [dict(e, session_id='other-session') for e in events]
        with self.assertRaises(AdmissionError):
            reconcile(measured_authorization(), foreign, run_id='a', clock=Clock(100600., 600., 'boot'), authorization_sha256='auth')
        self.assertEqual(session_accounting(measured_authorization(), foreign)['admitted_runs'], 0)

    def test_stale_active_session_pointer_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); (root / 'sessions' / 'session').mkdir(parents=True)
            a = measured_authorization(); atomic_json(root / 'sessions' / 'session' / 'authorization.json', a)
            atomic_json(root / 'ACTIVE_SESSION.json', {'authorization': 'sessions/session/authorization.json', 'session_id': 'session'})
            self.assertEqual(active_session(root, Clock(100100., 100., 'boot'))[1]['session_id'], 'session')
            with self.assertRaisesRegex(AdmissionError, 'final deadline'): active_session(root, Clock(128800., 28800., 'boot'))
            with self.assertRaisesRegex(AdmissionError, 'another boot'): active_session(root, Clock(100100., 100., 'reboot'))
            atomic_json(root / 'ACTIVE_SESSION.json', {'authorization': 'sessions/session/authorization.json', 'session_id': 'other'})
            with self.assertRaisesRegex(AdmissionError, 'disagrees'): active_session(root, Clock(100100., 100., 'boot'))
            atomic_json(root / 'ACTIVE_SESSION.json', {'authorization': 'sessions/gone/authorization.json', 'session_id': 'gone'})
            with self.assertRaises(AdmissionError): active_session(root, Clock(100100., 100., 'boot'))

    def test_overrun_is_charged_in_full_and_training_is_not_authorized(self):
        events = []; self._run(events, 'a', 300, 200, finish_after=330, status='FAIL')
        settled = self._settle(events, 'a', 600)
        self.assertEqual((settled['charged_seconds'], settled['overrun_seconds'], settled['released_reservation_seconds']), (330., 30., 0.))
        prereq = {k: 'x' for k in ['working_environment', 'controller_gap', 'evaluation_criterion', 'checkpoint_plan']}
        with self.assertRaisesRegex(AdmissionError, 'not authorized'):
            admission(measured_authorization(), events, category='training', seconds=100, name='t', justification='j', training_prerequisites=prereq, clock=Clock(100700., 700., 'boot'))

    def test_schema_v2_requires_a_declared_allowance(self):
        a = measured_authorization(); a['limits']['maximum_session_measured_launcher_seconds'] = None
        with self.assertRaises(AdmissionError): admission(a)
        a['limits']['maximum_session_measured_launcher_seconds'] = 27001.
        with self.assertRaises(AdmissionError): admission(a)
