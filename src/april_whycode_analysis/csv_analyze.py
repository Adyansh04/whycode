"""
CSV analysis & plotting for april/whycode yaw comparisons.

Usage:
  python3 csv_analyze.py --input /root/ros1_ws/src/whycon_whycode_localization/logs/data.csv \
                        --outdir /root/ros1_ws/src/whycon_whycode_localization/logs/analysis

Dependencies: pandas, numpy, matplotlib. Install with:
  pip3 install pandas numpy matplotlib
"""
import os
import io
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

plt.style.use("seaborn-deep")


def read_dirty_csv(path):
    # robust reader: skip lines that start with '//' or blank
    with open(path, "r", encoding="utf-8") as f:
        lines = [ln for ln in f.readlines() if not ln.strip().startswith("//")]
    txt = "".join(lines)
    # use pandas with flexible parsing; strip spaces from column names
    df = pd.read_csv(io.StringIO(txt), sep=",", skipinitialspace=True)
    df.columns = [c.strip() for c in df.columns]
    return df


def sanitize_cols(df):
    # normalize expected column names (lowercase without spaces)
    rename_map = {}
    for c in df.columns:
        cc = c.lower().replace(" ", "_")
        rename_map[c] = cc
    df = df.rename(columns=rename_map)
    # convert numeric columns
    for col in ["timestamp", "actual_yaw", "april_yaw_deg", "whycode_yaw_deg",
                "subtract_angle", "april_corrected", "whycode_corrected",
                "error_april", "error_whycode"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def wrap_to_180(angle_deg):
    """Wrap angle(s) to [-180, 180). Accept scalars or numpy arrays."""
    a = np.asarray(angle_deg)
    return (a + 180.0) % 360.0 - 180.0


def angular_diff_deg(a_deg, b_deg):
    """Compute a - b in shortest angular sense, result in degrees in [-180,180)."""
    return wrap_to_180(np.asarray(a_deg) - np.asarray(b_deg))


def circular_mean_deg(angles_deg):
    """Compute circular mean in degrees."""
    ang = np.deg2rad(angles_deg)
    s = np.nanmean(np.sin(ang))
    c = np.nanmean(np.cos(ang))
    return np.rad2deg(np.arctan2(s, c))


def circular_std_deg(angles_deg):
    """Approx circular std (in degrees) using resultant length."""
    ang = np.deg2rad(angles_deg)
    s = np.nanmean(np.sin(ang))
    c = np.nanmean(np.cos(ang))
    R = np.hypot(s, c)
    # avoid domain issues
    if R <= 0:
        return np.nan
    circ_var = 1 - R
    # approximate circular std in radians then degrees
    std_rad = np.sqrt(-2.0 * np.log(np.clip(R, 1e-12, 1.0)))
    return np.rad2deg(std_rad)


def compute_basic_stats(arr):
    arr = np.asarray(arr[~np.isnan(arr)])
    if arr.size == 0:
        return {}
    return {
        "count": int(arr.size),
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0,
        "median": float(np.median(arr)),
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
        "mae": float(np.mean(np.abs(arr))),
        "rmse": float(np.sqrt(np.mean(arr ** 2))),
    }


def ensure_outdir(d):
    os.makedirs(d, exist_ok=True)


def plot_time_series(df, outdir):
    fig, ax = plt.subplots(figsize=(10, 4))
    times = df["timestamp"] - df["timestamp"].min()
    if "april_corrected" in df.columns:
        ax.plot(times, wrap_to_180(df["april_corrected"]), ".", ms=3, label="april_corrected")
    if "whycode_corrected" in df.columns:
        ax.plot(times, wrap_to_180(df["whycode_corrected"]), ".", ms=3, label="whycode_corrected")
    if "actual_yaw" in df.columns:
        ax.plot(times, wrap_to_180(df["actual_yaw"]), "-", lw=1, color="k", alpha=0.6, label="actual_yaw")
    ax.set_xlabel("time (s, relative)")
    ax.set_ylabel("yaw (deg, wrapped)")
    ax.legend()
    ax.set_title("Yaw time series (wrapped)")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "time_series_yaw.png"), dpi=150)
    plt.close(fig)


def plot_errors_over_time(df, outdir):
    fig, ax = plt.subplots(figsize=(10, 4))
    times = df["timestamp"] - df["timestamp"].min()
    if "actual_yaw" in df.columns:
        if "april_corrected" in df.columns:
            e_apr = angular_diff_deg(df["april_corrected"], df["actual_yaw"])
            ax.plot(times, e_apr, label="april_error (deg)", marker=".", ms=3)
        if "whycode_corrected" in df.columns:
            e_why = angular_diff_deg(df["whycode_corrected"], df["actual_yaw"])
            ax.plot(times, e_why, label="whycode_error (deg)", marker=".", ms=3)
        ax.axhline(0, color="k", lw=0.8)
        ax.set_ylabel("angular error (deg)")
        ax.set_xlabel("time (s, relative)")
        ax.legend()
        ax.set_title("Angular errors vs time (shortest angle difference)")
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "errors_time.png"), dpi=150)
    plt.close(fig)


def plot_error_histograms(df, outdir):
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    # absolute error histograms (angular)
    if "actual_yaw" in df.columns and "april_corrected" in df.columns:
        e_apr = np.abs(angular_diff_deg(df["april_corrected"], df["actual_yaw"]))
        e_apr = np.asarray(e_apr)
        e_apr = e_apr[~np.isnan(e_apr)]
        axes[0].hist(e_apr, bins=50, color="C0", alpha=0.8)
        axes[0].set_title("April abs error histogram")
        axes[0].set_xlabel("|error| (deg)")
    if "actual_yaw" in df.columns and "whycode_corrected" in df.columns:
        e_why = np.abs(angular_diff_deg(df["whycode_corrected"], df["actual_yaw"]))
        e_why = np.asarray(e_why)
        e_why = e_why[~np.isnan(e_why)]
        axes[1].hist(e_why, bins=50, color="C1", alpha=0.8)
        axes[1].set_title("Whycode abs error histogram")
        axes[1].set_xlabel("|error| (deg)")
    # CDF combined
    ax2 = axes[2]
    for label, col, color in [
        ("april", "april_corrected", "C0"),
        ("whycode", "whycode_corrected", "C1")
    ]:
        if "actual_yaw" in df.columns and col in df.columns:
            vals = np.abs(angular_diff_deg(df[col], df["actual_yaw"]))
            vals = np.asarray(vals)
            vals = vals[~np.isnan(vals)]
            if vals.size:
                sorted_v = np.sort(vals)
                p = np.linspace(0, 1, sorted_v.size)
                ax2.plot(sorted_v, p, label=label, color=color)
    ax2.set_xlabel("|error| (deg)")
    ax2.set_ylabel("CDF")
    ax2.set_title("CDF of absolute angular errors")
    ax2.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "errors_histograms_cdf.png"), dpi=150)
    plt.close(fig)


def scatter_compare(df, outdir):
    if ("april_corrected" in df.columns) and ("whycode_corrected" in df.columns):
        fig, ax = plt.subplots(figsize=(5, 5))
        a = wrap_to_180(df["april_corrected"])
        w = wrap_to_180(df["whycode_corrected"])
        ax.scatter(a, w, s=8, alpha=0.6)
        mn = np.nanmin([a.min(), w.min()])
        mx = np.nanmax([a.max(), w.max()])
        ax.plot([mn, mx], [mn, mx], "k--", lw=0.8)
        ax.set_xlabel("april_corrected (deg, wrapped)")
        ax.set_ylabel("whycode_corrected (deg, wrapped)")
        ax.set_title("april vs whycode corrected yaw (wrapped)")
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "april_vs_whycode_scatter_wrapped.png"), dpi=150)
        plt.close(fig)


def bland_altman(df, outdir):
    # difference vs mean plot (use shortest-angle diff)
    if ("april_corrected" in df.columns) and ("whycode_corrected" in df.columns):
        a = wrap_to_180(df["april_corrected"].to_numpy())
        w = wrap_to_180(df["whycode_corrected"].to_numpy())
        valid = ~(np.isnan(a) | np.isnan(w))
        a = a[valid]; w = w[valid]
        mean = 0.5 * (a + w)
        diff = angular_diff_deg(a, w)  # april - why (shortest)
        md = np.mean(diff); sd = np.std(diff, ddof=1)
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.scatter(mean, diff, s=8, alpha=0.5)
        ax.axhline(md, color="k", label=f"mean diff={md:.3f}°")
        ax.axhline(md + 1.96 * sd, color="r", linestyle="--", label="+1.96sd")
        ax.axhline(md - 1.96 * sd, color="r", linestyle="--", label="-1.96sd")
        ax.set_xlabel("mean of april & whycode (deg, wrapped)")
        ax.set_ylabel("difference (april - whycode) deg (shortest)")
        ax.set_title("Bland-Altman (angular, shortest difference)")
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "bland_altman_angular.png"), dpi=150)
        plt.close(fig)


def rmse_over_time_window(df, outdir, window_s=1.0):
    # compute RMSE in sliding window (by timestamp seconds)
    if "actual_yaw" not in df.columns:
        return
    ts = df["timestamp"].to_numpy()
    start = ts.min(); end = ts.max()
    bins = np.arange(start, end + window_s, window_s)
    rows = []
    for b in bins:
        sel = df[(df["timestamp"] >= b) & (df["timestamp"] < b + window_s)]
        if sel.empty:
            continue

        # compute angular errors (may return numpy arrays); ensure NaNs removed
        apr_rmse = np.nan
        why_rmse = np.nan

        if "april_corrected" in sel.columns:
            apr_err = angular_diff_deg(sel["april_corrected"], sel["actual_yaw"])
            apr_err = np.asarray(apr_err)
            apr_err = apr_err[~np.isnan(apr_err)]
            if apr_err.size:
                apr_rmse = float(np.sqrt(np.mean((apr_err) ** 2)))

        if "whycode_corrected" in sel.columns:
            why_err = angular_diff_deg(sel["whycode_corrected"], sel["actual_yaw"])
            why_err = np.asarray(why_err)
            why_err = why_err[~np.isnan(why_err)]
            if why_err.size:
                why_rmse = float(np.sqrt(np.mean((why_err) ** 2)))

        rows.append({
            "t": b - start,
            "rmse_april": apr_rmse,
            "rmse_why": why_rmse
        })

    if not rows:
        return
    rdf = pd.DataFrame(rows)
    fig, ax = plt.subplots(figsize=(10, 4))
    if not rdf["rmse_april"].isna().all():
        ax.plot(rdf["t"], rdf["rmse_april"], label="rmse_april")
    if not rdf["rmse_why"].isna().all():
        ax.plot(rdf["t"], rdf["rmse_why"], label="rmse_why")
    ax.set_xlabel("time (s, relative)")
    ax.set_ylabel("RMSE (deg)")
    ax.legend()
    ax.set_title(f"RMSE over {window_s}s windows (angular shortest difference)")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "rmse_over_time.png"), dpi=150)
    plt.close(fig)


def plot_abs_error_box_by_actual_bins(df, outdir, n_bins=6):
    """Show how absolute angular error varies with ground-truth yaw (binned)."""
    if "actual_yaw" not in df.columns:
        return
    # create bins across wrapped actual yaw
    actual = wrap_to_180(df["actual_yaw"])
    bins = np.linspace(-180, 180, n_bins + 1)
    labels = [f"{int(bins[i])}..{int(bins[i+1])}" for i in range(n_bins)]
    # pd.cut accepts array-like; ensure we assign a categorical aligned with df length
    df["actual_bin"] = pd.cut(np.asarray(actual), bins=bins, labels=labels, include_lowest=True)
    fig, ax = plt.subplots(figsize=(10, 5))
    data = []
    labels2 = []
    if "april_corrected" in df.columns:
        for lbl in labels:
            sel = df[df["actual_bin"] == lbl]
            vals = np.abs(angular_diff_deg(sel["april_corrected"], sel["actual_yaw"]))
            vals = np.asarray(vals)
            vals = vals[~np.isnan(vals)]
            if vals.size:
                data.append(vals)
                labels2.append("APR " + lbl)
    if "whycode_corrected" in df.columns:
        for lbl in labels:
            sel = df[df["actual_bin"] == lbl]
            vals = np.abs(angular_diff_deg(sel["whycode_corrected"], sel["actual_yaw"]))
            vals = np.asarray(vals)
            vals = vals[~np.isnan(vals)]
            if vals.size:
                data.append(vals)
                labels2.append("WHY " + lbl)
    if data:
        ax.boxplot(data, labels=labels2, showfliers=False)
        ax.set_ylabel("|angular error| (deg)")
        ax.set_title("Absolute angular error by ground-truth yaw bins (APR / WHY per bin)")
        plt.xticks(rotation=45, ha="right")
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "abs_error_box_by_actual_bins.png"), dpi=150)
    plt.close(fig)


def circular_stats_summary(df, outdir):
    """Compute circular mean/std for corrected readings and for their residuals vs actual."""
    summary = {}
    if "april_corrected" in df.columns:
        arr = np.asarray(wrap_to_180(df["april_corrected"].dropna()))
        if arr.size:
            cm = circular_mean_deg(arr)
            cs = circular_std_deg(arr)
            summary["april_corrected_circ_mean_deg"] = cm
            summary["april_corrected_circ_std_deg"] = cs
    if "whycode_corrected" in df.columns:
        arr = np.asarray(wrap_to_180(df["whycode_corrected"].dropna()))
        if arr.size:
            cm = circular_mean_deg(arr)
            cs = circular_std_deg(arr)
            summary["whycode_corrected_circ_mean_deg"] = cm
            summary["whycode_corrected_circ_std_deg"] = cs
    # residuals if actual present
    if "actual_yaw" in df.columns and "april_corrected" in df.columns:
        resid = angular_diff_deg(df["april_corrected"], df["actual_yaw"])
        resid = np.asarray(resid)
        resid = resid[~np.isnan(resid)]
        if resid.size:
            summary["april_resid_circ_mean_deg"] = circular_mean_deg(resid)
            summary["april_resid_circ_std_deg"] = circular_std_deg(resid)
    if "actual_yaw" in df.columns and "whycode_corrected" in df.columns:
        resid = angular_diff_deg(df["whycode_corrected"], df["actual_yaw"])
        resid = np.asarray(resid)
        resid = resid[~np.isnan(resid)]
        if resid.size:
            summary["why_resid_circ_mean_deg"] = circular_mean_deg(resid)
            summary["why_resid_circ_std_deg"] = circular_std_deg(resid)
    # save to text
    with open(os.path.join(outdir, "circular_summary.txt"), "w") as f:
        for k, v in summary.items():
            f.write(f"{k}: {v}\n")
    return summary


def summary_and_save(df, outdir):
    # recompute residuals to be safe
    if "actual_yaw" in df.columns:
        if "april_corrected" in df.columns:
            df["resid_april"] = angular_diff_deg(df["april_corrected"], df["actual_yaw"])
        if "whycode_corrected" in df.columns:
            df["resid_why"] = angular_diff_deg(df["whycode_corrected"], df["actual_yaw"])
    # basic stats
    stats = {}
    if "resid_april" in df.columns:
        stats["april"] = compute_basic_stats(df["resid_april"].to_numpy())
    if "resid_why" in df.columns:
        stats["whycode"] = compute_basic_stats(df["resid_why"].to_numpy())
    # save summary
    with open(os.path.join(outdir, "summary.txt"), "w") as f:
        f.write("Summary statistics (errors = corrected - actual, shortest-angle)\n\n")
        for k, v in stats.items():
            f.write(f"--- {k} ---\n")
            for kk, vv in v.items():
                f.write(f"{kk}: {vv}\n")
            f.write("\n")
    # save processed CSV
    df.to_csv(os.path.join(outdir, "processed.csv"), index=False)
    return stats


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--input", "-i", required=True)
    p.add_argument("--outdir", "-o", default="./analysis")
    p.add_argument("--window", type=float, default=1.0, help="RMSE time window seconds")
    args = p.parse_args()

    ensure_outdir(args.outdir)
    df = read_dirty_csv(args.input)
    df = sanitize_cols(df)

    # require timestamp column
    if "timestamp" not in df.columns:
        raise SystemExit("CSV must contain a 'timestamp' column")
    # convert timestamp to numeric seconds if necessary
    df["timestamp"] = pd.to_numeric(df["timestamp"], errors="coerce")

    # If april_corrected / whycode_corrected missing, try to derive from april_yaw_deg and subtract_angle
    if "april_corrected" not in df.columns and "april_yaw_deg" in df.columns and "subtract_angle" in df.columns:
        df["april_corrected"] = df["april_yaw_deg"] - df["subtract_angle"]
    if "whycode_corrected" not in df.columns and "whycode_yaw_deg" in df.columns and "subtract_angle" in df.columns:
        df["whycode_corrected"] = df["whycode_yaw_deg"] - df["subtract_angle"]

    # normalize yaw columns to numeric and wrap where useful
    for col in ["april_corrected", "whycode_corrected", "actual_yaw"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    # compute / recompute error columns (shortest-angle)
    if "actual_yaw" in df.columns:
        if "april_corrected" in df.columns:
            df["error_april"] = angular_diff_deg(df["april_corrected"], df["actual_yaw"])
        if "whycode_corrected" in df.columns:
            df["error_whycode"] = angular_diff_deg(df["whycode_corrected"], df["actual_yaw"])

    # existing plots (time series, errors, histograms, scatter, bland-altman, rmse)
    plot_time_series(df, args.outdir)
    plot_errors_over_time(df, args.outdir)
    plot_error_histograms(df, args.outdir)
    scatter_compare(df, args.outdir)
    bland_altman(df, args.outdir)
    rmse_over_time_window(df, args.outdir, window_s=args.window)

    # Additional angle-focused plots and summaries
    plot_abs_error_box_by_actual_bins(df, args.outdir, n_bins=6)
    circ_summary = circular_stats_summary(df, args.outdir)

    stats = summary_and_save(df, args.outdir)

    # print brief summary
    print("Saved analysis to:", os.path.abspath(args.outdir))
    for name, s in stats.items():
        print(f"{name} RMSE: {s.get('rmse', 'n/a'):.3f} deg, bias (mean): {s.get('mean', 'n/a'):.3f}")
    # print circular summary
    print("Circular stats saved to circular_summary.txt")
    for k, v in circ_summary.items():
        print(f"{k}: {v:.3f}")


if __name__ == "__main__":
    main()