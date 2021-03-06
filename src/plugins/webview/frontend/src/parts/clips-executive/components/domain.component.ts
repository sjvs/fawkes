
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit, OnDestroy } from '@angular/core';
import { interval, forkJoin } from 'rxjs';
import { map } from 'rxjs/operators';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';

import { ClipsExecutiveApiService } from '../services/api.service';


@Component({
  selector: 'ff-clips-executive-domain',
  templateUrl: './domain.component.html',
  styleUrls: ['./domain.component.scss']
})
export class DomainComponent implements OnInit, OnDestroy {

  constructor(private api_service: ClipsExecutiveApiService,
              private backendcfg: BackendConfigurationService) {}

  private backend_subscription = null;

  loading = false;

  domain_data = null;
  auto_refresh_subscription = null;

  zero_message_facts      = 'No facts received';
  zero_message_predicates = 'No predicates received';
  zero_message_objects    = 'No objects received';
  zero_message_operators  = 'No operators received';

  displayed_fact_columns   = [ 'fact_name', 'fact_params' ];
  displayed_object_columns = [ 'object_name', 'object_type' ];
  displayed_op_columns = [ 'op_name', 'op_params', 'op_wait_sensed' ];
  displayed_pred_columns = [ 'pred_name', 'pred_params', 'pred_sensed' ];

  ngOnInit() {
    this.reset_domain_data();
    this.refresh_domain_data();

    this.backend_subscription = this.backendcfg.backend_changed.subscribe((b) => {
      this.reset_domain_data();
      this.refresh_domain_data();
    });
  }

  ngOnDestroy() {
    this.backend_subscription.unsubscribe();
    this.backend_subscription = null;
    this.disable_autorefresh();
  }

  reset_domain_data() {
    this.domain_data = {
      operators:      {},
      operators_arr:  [],
      predicates:     {},
      predicates_arr: [],
      objects:        [],
      facts:          []
    };
  }

  refresh_domain_data() {
    this.loading = true;
    forkJoin([
      // forkjoin here to allow for requesting multiple items
      this.api_service.list_domain_operators(),
      this.api_service.list_domain_predicates(),
      this.api_service.list_domain_objects(),
      this.api_service.list_domain_facts(),
    ])
      .pipe(
        map((data: any[]) => {
          return { operators:      data[0].reduce((m, obj) => (m[obj.name] = obj, m), {}),
                   operators_arr:  data[0],
                   predicates:     data[1].reduce((m, obj) => (m[obj.name] = obj, m), {}),
                   predicates_arr: data[1],
                   objects:        data[2],
                   facts:          data[3].sort((a, b) => (a.name < b.name))};
        })
      )
      .subscribe(
        (dd) => {
          console.log(`Received domain data: ${Object.keys(dd.operators).length} operators, ` +
                      `${Object.keys(dd.predicates).length} predicates, ${dd.objects.length} objects, ` +
                      `${dd.facts.length} facts`);
          this.domain_data = dd;
          this.loading = false;
          if (dd.facts.length === 0) {
            this.zero_message_facts = 'No facts in current domain';
          }
          if (dd.predicates_arr.length === 0) {
            this.zero_message_predicates = 'No predicates in current domain';
          }
          if (dd.operators_arr.length === 0) {
            this.zero_message_operators = 'No operators in current domain';
          }
          if (dd.objects.length === 0) {
            this.zero_message_objects = 'No objects in current domain';
          }
        },
        (err) => {
          this.loading = false;

          this.zero_message_facts = this.zero_message_predicates =
            this.zero_message_operators = this.zero_message_objects =
            (err.status === 0)
            ? 'API server unavailable. Robot down?'
            : `Failed to retrieve domain info: ${err.error}`;
        }
      );
  }

  private enable_autorefresh() {
    if (this.auto_refresh_subscription) {  return; }
    this.auto_refresh_subscription =
      interval(2000).subscribe((num) => {
        this.refresh_domain_data();
      });
    this.refresh_domain_data();
  }

  private disable_autorefresh() {
    if (this.auto_refresh_subscription) {
      this.auto_refresh_subscription.unsubscribe();
      this.auto_refresh_subscription = null;
    }
  }

  toggle_autorefresh() {
    if (this.auto_refresh_subscription) {
      this.disable_autorefresh();
    } else {
      this.enable_autorefresh();
    }
  }

}
