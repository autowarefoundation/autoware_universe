#include <autoware/camp_selector/adapter_contracts.hpp>
#include <autoware/camp_selector/camp_ranker.hpp>

#include <vector>

int main(int argc, char ** argv)
{
  if (argc != 2) return 2;
  using namespace autoware::camp_selector;
  const auto model = load_camp_fixed_weight_model(argv[1]);
  CampStatusPattern status;
  status.fill(CampAtomStatus::Observed);
  std::vector<CampAtomVector> atoms(model.candidate_pool_k, model.scales);
  atoms.back().fill(0.0);
  const auto result = rank_camp_candidates(model, status, atoms);
  PublishedPlanLedger<int> ledger;
  const auto id = ledger.stage(1.0, {0, 7});
  if (!ledger.confirm_published(id, 1) || ledger.previous()->states != 7) return 3;
  return result.selected_index == model.candidate_pool_k - 1 &&
             result.costs.size() == model.candidate_pool_k
           ? 0
           : 1;
}
