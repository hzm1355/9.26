#!/usr/bin/env bash
# 小探针：确认 grep -E 的转义方括号模式到底怎么匹配（pk158 反向验证的点名判据用它）。
printf "  FAIL RESULT['followed'] == EtcQuery.Request.RESULT_FOLLOWED=4  3\n" > /tmp/pk159_pat.txt
cat -A /tmp/pk159_pat.txt | head -2
for P in 'FAIL.*RESULT.\[.followed' 'FAIL.*RESULT.followed' 'FAIL.*RESULT' 'RESULT.\[' ; do
  if grep -qE "$P" /tmp/pk159_pat.txt; then
    echo "MATCH   $P"
  else
    echo "NOMATCH $P"
  fi
done
