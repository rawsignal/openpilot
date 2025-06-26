/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/offroad/settings/software_panel.h"

SoftwarePanelSP::SoftwarePanelSP(QWidget *parent) : SoftwarePanel(parent) {
  // branch selector
  QObject::disconnect(targetBranchBtn, nullptr, nullptr, nullptr);
  connect(targetBranchBtn, &ButtonControlSP::clicked, [=]() {
    InputDialog d(tr("Search Branch"), this, tr("Enter search keywords, or leave blank to list all branches."), false);
      d.setMinLength(0);
      const int ret = d.exec();
      if (ret) {
        searchBranches(d.text());
      }
  });

  // add update button
  updateBtn = new ButtonControlSP(tr("Force Update"), tr("UPDATE"), tr(""));
  connect(updateBtn, &ButtonControlSP::clicked, this, &SoftwarePanelSP::triggerUpdate);
  addItem(updateBtn);
}

/**
 * @brief Searches for available branches based on a query string, presents the results in a dialog,
 * and updates the target branch if a selection is made.
 *
 * This function filters the list of branches based on the provided query, and displays the filtered branches in a selection dialog.
 * If a branch is selected, the "UpdaterTargetBranch" parameter is updated and a check for updates is triggered.
 * If no branches are found matching the query, an alert dialog is displayed.
 *
 * @param query The search query string.
 */
void SoftwarePanelSP::searchBranches(const QString &query) {

  QStringList branches = QString::fromStdString(params.get("UpdaterAvailableBranches")).split(",");
  QStringList results = searchFromList(query, branches);
  results.sort();

  if (results.isEmpty()) {
    ConfirmationDialog::alert(tr("No branches found for keywords: %1").arg(query), this);
    return;
  }

  QString selected_branch = MultiOptionDialog::getSelection(tr("Select a branch"), results, "", this);

  if (!selected_branch.isEmpty()) {
    params.put("UpdaterTargetBranch", selected_branch.toStdString());
    targetBranchBtn->setValue(selected_branch);
    checkForUpdates();
  }
}

void SoftwarePanelSP::triggerUpdate() {
  QString message = tr("Are you sure you want to force an update?\n\nOpenpilot will restart.");
  if (ConfirmationDialog::confirm(message, tr("Update"), this))
  {
    std::string cmd = "op switch " + params.get("UpdaterTargetBranch") + " && op start";
    std::system(cmd.c_str());
    // openpilot will restart if the process is successful. If not, show an error dialog.
    ConfirmationDialog::alert(tr("Failed to apply updates."), this);
  }
}

void SoftwarePanelSP::showEvent(QShowEvent *event) {
  SoftwarePanel::showEvent(event);
  updateBtn->setEnabled(params.getBool("IsOffroad"));
}
