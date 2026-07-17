package jp.mycobotlab.handteleop;

import android.app.Activity;
import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;

/**
 * 2D ランチャー。タップされると Oculus Browser で
 * http://localhost:8001/hand を開き、自身は即終了する。
 *
 * localhost は各 Quest 側で `adb reverse tcp:8001 tcp:8001` により
 * PC の hand server (:8001) へ転送される前提（deploy_hand.py が設定）。
 */
public class LauncherActivity extends Activity {

    private static final String TAG = "HandTeleopLauncher";
    private static final String TARGET_URL = "http://localhost:8001/hand";
    private static final String OCULUS_BROWSER_PKG = "com.oculus.browser";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Uri uri = Uri.parse(TARGET_URL);

        // 1st try: Oculus Browser を明示指定して開く
        Intent intent = new Intent(Intent.ACTION_VIEW, uri);
        intent.setPackage(OCULUS_BROWSER_PKG);
        intent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);

        try {
            startActivity(intent);
        } catch (ActivityNotFoundException e) {
            // Oculus Browser が無い/解決できない場合は既定ブラウザにフォールバック
            Log.w(TAG, "com.oculus.browser not found, retrying without package", e);
            Intent fallback = new Intent(Intent.ACTION_VIEW, uri);
            fallback.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
            try {
                startActivity(fallback);
            } catch (ActivityNotFoundException e2) {
                Log.e(TAG, "No browser available to handle " + TARGET_URL, e2);
                Toast.makeText(this, "ブラウザが見つかりません", Toast.LENGTH_LONG).show();
            }
        }

        // ランチャー自身は残さず終了する
        finish();
    }
}
