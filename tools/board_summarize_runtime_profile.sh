#!/bin/sh
set -u

INPUT=${1:-/userdata/robot_slamd_board_runtime_profile_smoke/latest}
if [ -d "$INPUT" ]; then
    CSV="$INPUT/runtime_profile.csv"
else
    CSV="$INPUT"
fi

if [ ! -s "$CSV" ]; then
    echo "runtime_profile_csv,missing"
    exit 1
fi

awk -F, '
NR > 1 {
    n++;
    dur = $2;
    cpu += $4;
    if ($4 > cpumax) cpumax = $4;
    rss = $5 / 1024.0;
    rsssum += rss;
    if (rss > rssmax) rssmax = rss;
    if ($7 > thrmax) thrmax = $7;
}
END {
    if (n <= 0) {
        print "strict,fail_empty";
        exit 1;
    }
    printf "duration_s,%.3f\n", dur;
    printf "cpu_percent_mean,%.3f\n", cpu / n;
    printf "cpu_percent_max,%.3f\n", cpumax;
    printf "rss_mb_mean,%.3f\n", rsssum / n;
    printf "rss_mb_max,%.3f\n", rssmax;
    printf "thread_count_max,%d\n", thrmax;
    if (dur > 0 && rssmax < 200 && cpumax < 200) {
        print "strict,pass";
    } else {
        print "strict,fail";
        exit 1;
    }
}
' "$CSV"
