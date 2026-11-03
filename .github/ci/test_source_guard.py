"""Exercise the workflow's revision gate without network access or robot code."""
import ast
from pathlib import Path
import re
import textwrap
import unittest


def load_guard():
    workflow = (Path(__file__).resolve().parents[1]/'workflows/robot-ci.yml').read_text()
    code = workflow.split("python3 - <<'PY'\n", 1)[1].split('\n          PY', 1)[0]
    tree = ast.parse(textwrap.dedent(code))
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'verify_source')
    namespace = {'re':re}
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<workflow source guard>', 'exec'), namespace)
    return namespace['verify_source']


class SourceGuardTests(unittest.TestCase):
    def setUp(self):
        self.verify = load_guard()
        self.source = {'head':'a'*40,'base':'b'*40,'merge':'c'*40}
        self.args = ['a'*40,'b'*40,'c'*40,'b'*40,
                     'Reedy-Creek-Robotics/BBTC-2025','15707_Main_Coding_2026-2027']

    def test_current_metadata_passes(self):
        self.verify(self.source, *self.args)

    def test_stale_event_base_does_not_reject_a_current_merge(self):
        # Exact shape of PR #35's failure: event old, merge parent and branch new.
        self.args[1] = 'd'*40
        self.verify(self.source, *self.args)

    def test_old_merge_still_fails_after_destination_advances(self):
        self.args[3] = 'd'*40
        with self.assertRaisesRegex(SystemExit,'Destination branch changed'):
            self.verify(self.source, *self.args)

    def test_wrong_head_merge_repo_branch_and_malformed_hash_are_rejected(self):
        for index,value in ((0,'d'*40),(2,'d'*40),(4,'other/repo'),(5,'master'),(1,'not-a-hash')):
            args = self.args.copy(); args[index] = value
            with self.subTest(index=index), self.assertRaises(SystemExit):
                self.verify(self.source, *args)

    def test_actual_pr35_revisions(self):
        source = {'head':'e18c2f7982b70976f32412f4ee004dd3f8891f12',
                  'base':'3d9554e6f056faa5c9745b571869083f3e42839f',
                  'merge':'1af6dfa28f1f7ef5db423f06b99acc6204cffadb'}
        self.verify(source, source['head'], '542996576e4709baba0d18b293b591cb4ae5920e',
                    source['merge'], source['base'], self.args[4], self.args[5])


if __name__ == '__main__':
    unittest.main()
